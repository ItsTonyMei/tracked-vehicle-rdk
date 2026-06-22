#!/usr/bin/env python3
"""
本地屏显节点 — 在 RDK X5 连接的屏幕上显示相机画面和检测结果

订阅:
  /image                              — JPEG 图像 (hobot_codec)
  /hobot_mono2d_body_detection        — 人体检测结果 (PerceptionTargets)

用法:
  ros2 run tracked_vehicle display_node
  ros2 run tracked_vehicle display_node --ros-args -p show_depth:=true
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ai_msgs.msg import PerceptionTargets
import cv2
import numpy as np


class DisplayNode(Node):
    def __init__(self):
        super().__init__('display_node')

        self.show_depth = self.declare_parameter('show_depth', False).value
        self.target_dist = self.declare_parameter('target_dist', 2.0).value
        self.bbox_ref = self.declare_parameter('bbox_ref_width', 500.0).value
        self.bbox_ref_dist = self.declare_parameter('bbox_ref_dist', 2.0).value

        self._frame = None
        self._targets = None
        self._window = 'Tracked Vehicle — RDK X5'

        # 全屏窗口
        cv2.namedWindow(self._window, cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty(self._window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        self.sub_img = self.create_subscription(Image, '/image', self.img_cb, 10)
        self.sub_det = self.create_subscription(
            PerceptionTargets, '/hobot_mono2d_body_detection', self.det_cb, 10)

        # 渲染循环 ~20fps
        self.timer = self.create_timer(0.05, self.render)
        self.get_logger().info('display_node 就绪 (1024x600 全屏)')

    def img_cb(self, msg: Image):
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        self._frame = cv2.imdecode(raw, cv2.IMREAD_COLOR)

    def det_cb(self, msg: PerceptionTargets):
        self._targets = msg

    def render(self):
        if self._frame is None:
            return

        frame = self._frame.copy()
        h, w = frame.shape[:2]

        targets = self._targets
        if targets is not None:
            for t in targets.targets:
                if t.type != 'person':
                    continue

                for roi in t.rois:
                    r = roi.rect
                    x1, y1 = int(r.x_offset), int(r.y_offset)
                    x2, y2 = x1 + int(r.width), y1 + int(r.height)

                    # 距离估算
                    if r.width > 0:
                        dist = (self.bbox_ref * self.bbox_ref_dist) / r.width
                    else:
                        dist = 0

                    # 主框
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # 头部区域 (上 1/3)
                    head_h = (y2 - y1) // 3
                    cv2.rectangle(frame, (x1, y1), (x2, y1 + head_h), (255, 255, 0), 2)

                    # 标签
                    label = f'#{t.track_id} {dist:.1f}m'
                    cv2.rectangle(frame, (x1, y1 - 24), (x1 + 160, y1), (0, 255, 0), -1)
                    cv2.putText(frame, label, (x1 + 4, y1 - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

                    # 十字准心 (画面中心)
                    cx, cy = w // 2, h // 2
                    cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (128, 128, 128), 1)
                    cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (128, 128, 128), 1)

                    # 人框中心到画面中心的偏移线
                    bx, by = x1 + int(r.width) // 2, y1 + int(r.height) // 2
                    cv2.line(frame, (bx, by), (cx, cy), (255, 0, 255), 1)

                    # 死区圈
                    deadzone_px = 40
                    cv2.ellipse(frame, (cx, cy), (deadzone_px, deadzone_px),
                                0, 0, 360, (100, 100, 100), 1)

        # 状态条
        status = 'STOP'
        color = (0, 0, 255)
        if targets is not None and targets.targets:
            status = f'TRACK #{targets.targets[0].track_id} | {targets.fps}FPS'
            color = (0, 255, 0)
        cv2.putText(frame, status, (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        cv2.imshow(self._window, frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    rclpy.spin(DisplayNode())
    rclpy.shutdown()
