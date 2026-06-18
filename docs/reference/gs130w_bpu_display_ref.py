#!/usr/bin/env python3
"""
GS130W 双目摄像头 → BPU YOLOv5s 推理 → HDMI DRM 显示
桥接 ROS2 mipi_cam 节点输出与 srcampy Display + BPU 推理
"""

import os
import sys
import signal
import argparse
import threading
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from hobot_vio import libsrcampy as srcampy
except ImportError:
    from hobot_vio_rdkx5 import libsrcampy as srcampy

import hbm_runtime

sys.path.append("/app/pydev_demo")
import utils.preprocess_utils as pre_utils
import utils.postprocess_utils as post_utils
import utils.common_utils as common
import utils.draw_utils as draw

STRIDES = np.array([8, 16, 32], dtype=np.int32)
ANCHORS = np.array([
    [10, 13], [16, 30], [33, 23],
    [30, 61], [62, 45], [59, 119],
    [116, 90], [156, 198], [373, 326]
], dtype=np.float32).reshape(3, 3, 2)


class YoloV5X:
    def __init__(self, model_path, score_thres=0.25, nms_thres=0.45):
        self.model = hbm_runtime.HB_HBMRuntime(model_path)
        self.model_name = self.model.model_names[0]
        self.input_names = self.model.input_names[self.model_name]
        self.output_names = self.model.output_names[self.model_name]
        self.input_shapes = self.model.input_shapes[self.model_name]
        self.output_quants = self.model.output_quants[self.model_name]
        self.input_H = self.input_shapes[self.input_names[0]][2]
        self.input_W = self.input_shapes[self.input_names[0]][3]
        self.score_thres = score_thres
        self.nms_thres = nms_thres
        self.resize_type = 1
        self.classes_num = 80

    def pre_process(self, nv12_bytes, original_width, original_height):
        y, uv = pre_utils.split_nv12_bytes(nv12_bytes, original_width, original_height)
        y_resized, uv_resized = pre_utils.resize_nv12_yuv(y, uv, self.input_H, self.input_W)
        y_input = y_resized[..., None][None, ...]
        uv_input = uv_resized[None, ...]
        nv12 = np.concatenate((y_input.reshape(-1), uv_input.reshape(-1)), axis=0)
        nv12 = nv12.reshape((1, self.input_H * 3 // 2, self.input_W, 1))
        return {self.model_name: {self.input_names[0]: nv12}}

    def forward(self, input_tensor):
        outputs = self.model.run(input_tensor)
        return outputs[self.model_name]

    def post_process(self, outputs, img_w, img_h):
        fp32_outputs = post_utils.dequantize_outputs(outputs, self.output_quants)
        pred = post_utils.decode_outputs(self.output_names, fp32_outputs,
                                         STRIDES, ANCHORS, self.classes_num)
        xyxy_boxes, score, cls = post_utils.filter_predictions(pred, self.score_thres)
        keep = post_utils.NMS(xyxy_boxes, score, cls, self.nms_thres)
        xyxy = post_utils.scale_coords_back(xyxy_boxes[keep], img_w, img_h,
                                            self.input_W, self.input_H, self.resize_type)
        return xyxy, cls[keep], score[keep]


class GS130WDisplayNode(Node):
    def __init__(self, disp, yolov5x, coco_names, disp_w, disp_h):
        super().__init__('gs130w_display_node')
        self.disp = disp
        self.yolov5x = yolov5x
        self.coco_names = coco_names
        self.disp_w = disp_w
        self.disp_h = disp_h
        self.latest_frame = None
        self.lock = threading.Lock()
        self.sub = self.create_subscription(
            Image, '/image_combine_raw', self.image_callback, 10)
        self.get_logger().info('GS130W Display Node started')

    def image_callback(self, msg):
        with self.lock:
            self.latest_frame = bytes(msg.data)

    def run_loop(self):
        rate = self.create_rate(30)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)
            with self.lock:
                frame = self.latest_frame
            if frame is None:
                rate.sleep()
                continue

            img_w, img_h = 816, 1920

            self.disp.set_img(frame, img_w, img_h, chn=0)

            input_tensor = self.yolov5x.pre_process(frame, img_w, img_h)
            outputs = self.yolov5x.forward(input_tensor)
            boxes, cls_ids, scores = self.yolov5x.post_process(outputs, img_w, img_h)

            draw.draw_detections_on_disp(
                self.disp, boxes, cls_ids, scores, self.coco_names,
                common.rdk_colors, chn=2,
                img_w=img_w, img_h=img_h,
                disp_w=self.disp_w, disp_h=self.disp_h)

            rate.sleep()


def get_display_res():
    if not os.path.exists("/usr/bin/get_hdmi_res"):
        return 1920, 1080
    import subprocess
    p = subprocess.Popen(["/usr/bin/get_hdmi_res"], stdout=subprocess.PIPE)
    result = p.communicate()
    res = result[0].split(b',')
    res[1] = max(min(int(res[1]), 1920), 0)
    res[0] = max(min(int(res[0]), 1080), 0)
    return int(res[1]), int(res[0])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model-path', type=str,
                        default='/opt/hobot/model/x5/basic/yolov5s_672x672_nv12.bin')
    parser.add_argument('--score-thres', type=float, default=0.25)
    parser.add_argument('--nms-thres', type=float, default=0.45)
    parser.add_argument('--label-file', type=str,
                        default='/app/pydev_demo/08_mipi_camera_sample/coco_classes.names')
    opt = parser.parse_args()

    rclpy.init(args=sys.argv)

    disp_w, disp_h = get_display_res()
    print(f"Display: {disp_w}x{disp_h}")

    disp = srcampy.Display()
    disp.display(0, disp_w, disp_h)

    yolov5x = YoloV5X(opt.model_path, opt.score_thres, opt.nms_thres)
    coco_names = common.load_class_names(opt.label_file)

    node = GS130WDisplayNode(disp, yolov5x, coco_names, disp_w, disp_h)

    try:
        node.run_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        disp.close()
        print("Done.")


if __name__ == '__main__':
    main()
