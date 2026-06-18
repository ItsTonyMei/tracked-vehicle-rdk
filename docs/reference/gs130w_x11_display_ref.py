#!/usr/bin/env python3
"""
GS130W 双目摄像头 → BPU YOLOv5s 推理 → HDMI 显示 (via X11 + OpenCV)
v4: 优化版
  - resize+pad 替代 cv2.rotate（省 34ms）
  - 隔帧跳过后处理（省 95ms/2帧）
  - 目标帧率: ~8fps (from 4-5fps)
"""

import os, sys, time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

sys.path.append("/app/pydev_demo")
import utils.preprocess_utils as pre_utils
import utils.postprocess_utils as post_utils
import utils.common_utils as common
import utils.draw_utils as draw_utils
import hbm_runtime

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


def nv12_to_bgr(nv12_bytes, width, height):
    y_size = width * height
    y = np.frombuffer(nv12_bytes[:y_size], dtype=np.uint8).reshape(height, width)
    uv = np.frombuffer(nv12_bytes[y_size:], dtype=np.uint8).reshape(height // 2, width)
    yuv = np.empty((height + height // 2, width), dtype=np.uint8)
    yuv[:height] = y
    yuv[height:] = uv
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)


def portrait_to_display(bgr, display_w=1920, display_h=1080):
    """Scale portrait BGR to fit display height, pad to display width."""
    h, w = bgr.shape[:2]
    scale = display_h / h
    new_w = int(w * scale)
    scaled = cv2.resize(bgr, (new_w, display_h))
    pad_left = (display_w - new_w) // 2
    return cv2.copyMakeBorder(scaled, 0, 0, pad_left, display_w - new_w - pad_left,
                              cv2.BORDER_CONSTANT, value=[0, 0, 0])


class GS130WNode(Node):
    def __init__(self):
        super().__init__('gs130w_display_node')
        self.latest_frame = None
        self.frame_count = 0
        self.sub = self.create_subscription(
            Image, '/image_combine_raw', self.image_callback, 10)

    def image_callback(self, msg):
        self.latest_frame = bytes(msg.data)
        self.frame_count += 1


def main():
    os.environ['DISPLAY'] = ':0'
    rclpy.init(args=sys.argv)

    model_path = '/opt/hobot/model/x5/basic/yolov5s_672x672_nv12.bin'
    yolov5x = YoloV5X(model_path)
    coco_names = common.load_class_names('/app/pydev_demo/08_mipi_camera_sample/coco_classes.names')

    node = GS130WNode()

    print("Waiting for first frame...", flush=True)
    while rclpy.ok() and node.latest_frame is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f"Got first frame: {len(node.latest_frame)} bytes", flush=True)

    cv2.namedWindow('GS130W YOLOv5s', cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty('GS130W YOLOv5s', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    last_count = node.frame_count
    fps_timer = time.time()
    fps_frames = 0
    frame_idx = 0

    cached_boxes = np.zeros((0, 4), dtype=np.float32)
    cached_cls = np.zeros(0, dtype=np.int32)
    cached_scores = np.zeros(0, dtype=np.float32)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)

            if node.latest_frame is None or node.frame_count == last_count:
                time.sleep(0.002)
                continue

            last_count = node.frame_count
            fps_frames += 1
            frame_idx += 1
            frame = node.latest_frame
            img_w, img_h = 816, 1920

            bgr = nv12_to_bgr(frame, img_w, img_h)

            input_tensor = yolov5x.pre_process(frame, img_w, img_h)
            outputs = yolov5x.forward(input_tensor)

            if frame_idx % 2 == 1:
                boxes, cls_ids, scores = yolov5x.post_process(outputs, img_w, img_h)
                cached_boxes, cached_cls, cached_scores = boxes, cls_ids, scores
            else:
                boxes, cls_ids, scores = cached_boxes, cached_cls, cached_scores

            bgr = draw_utils.draw_boxes(bgr, boxes, cls_ids, scores, coco_names, common.rdk_colors)

            display = portrait_to_display(bgr)

            cv2.imshow('GS130W YOLOv5s', display)
            cv2.waitKey(1)

            if fps_frames >= 30:
                elapsed = time.time() - fps_timer
                fps = fps_frames / elapsed if elapsed > 0 else 0
                print(f"FPS: {fps:.1f}, detections: {len(boxes)}", flush=True)
                fps_frames = 0
                fps_timer = time.time()

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
        print("Done.")


if __name__ == '__main__':
    main()
