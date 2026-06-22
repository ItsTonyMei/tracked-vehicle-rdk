#!/usr/bin/env python3
"""本地屏显 - 订阅 CompressedImage (JPEG) + 检测结果"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ai_msgs.msg import PerceptionTargets
import cv2
import numpy as np

class DisplayNode(Node):
    def __init__(self):
        super().__init__('display_node')
        self.target_dist = self.declare_parameter('target_dist', 2.0).value
        self.bbox_ref = self.declare_parameter('bbox_ref_width', 500.0).value
        self.bbox_ref_dist = self.declare_parameter('bbox_ref_dist', 2.0).value
        self.rotate_deg = self.declare_parameter('rotate_deg', 0).value
        self._frame = None
        self._targets = None
        self._window = 'RDK X5 Tracker'
        self._init_display()

    def _init_display(self):
        cv2.namedWindow(self._window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._window, 1024, 600)
        cv2.moveWindow(self._window, 0, 0)
        cv2.setWindowProperty(self._window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        self.get_logger().info(f'display_node OK (rotate={self.rotate_deg}deg)')

    def img_cb(self, msg: CompressedImage):
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        self._frame = cv2.imdecode(raw, cv2.IMREAD_COLOR)

    def det_cb(self, msg: PerceptionTargets):
        self._targets = msg

    def _rotate(self, img):
        if self.rotate_deg == 90:
            return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotate_deg == 270 or self.rotate_deg == -90:
            return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        elif self.rotate_deg == 180:
            return cv2.rotate(img, cv2.ROTATE_180)
        return img

    def render(self):
        if self._frame is None:
            return
        # 原始帧 (mipi_cam 已做 rotation=90, 输出 960x544)
        frame = self._frame.copy()
        orig_h, orig_w = frame.shape[:2]

        # ── 先画检测框 (在原图坐标系) ─────────────────
        targets = self._targets
        if targets is not None:
            for t in targets.targets:
                if t.type != 'person':
                    continue
                # 收集各类型 ROI
                body_roi = head_roi = None
                for roi in t.rois:
                    if roi.type == 'body':
                        body_roi = roi.rect
                    elif roi.type == 'head':
                        head_roi = roi.rect

                if body_roi is None:
                    continue

                r = body_roi
                x1, y1 = int(r.x_offset), int(r.y_offset)
                x2, y2 = x1 + int(r.width), y1 + int(r.height)
                dist = (self.bbox_ref * self.bbox_ref_dist) / r.width if r.width > 0 else 0

                # 身体框 (绿色)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # 头部框: 优先用模型检测的 head ROI, 否则用 body 上 1/3 估算
                if head_roi is not None:
                    hx1, hy1 = int(head_roi.x_offset), int(head_roi.y_offset)
                    hx2, hy2 = hx1 + int(head_roi.width), hy1 + int(head_roi.height)
                else:
                    hx1, hy1 = x1, y1
                    hx2, hy2 = x2, y1 + (y2 - y1) // 3
                cv2.rectangle(frame, (hx1, hy1), (hx2, hy2), (255, 255, 0), 2)

                # 标签
                label = f'#{t.track_id} {dist:.1f}m'
                cv2.rectangle(frame, (x1, y1 - 24), (x1 + 160, y1), (0, 255, 0), -1)
                cv2.putText(frame, label, (x1 + 4, y1 - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

                # 人框→中心偏移线
                bx, by = x1 + int(r.width) // 2, y1 + int(r.height) // 2
                cv2.line(frame, (bx, by), (orig_w//2, orig_h//2), (255, 0, 255), 1)

        # ── 画面中心十字 + 死区 (固定, 只画一次) ─────
        if self._frame is not None:
            cx0, cy0 = orig_w // 2, orig_h // 2
            cv2.line(frame, (cx0 - 20, cy0), (cx0 + 20, cy0), (128, 128, 128), 1)
            cv2.line(frame, (cx0, cy0 - 20), (cx0, cy0 + 20), (128, 128, 128), 1)
            cv2.ellipse(frame, (cx0, cy0), (40, 40), 0, 0, 360, (100, 100, 100), 1)

        # ── 可选旋转 ─────────────────────────────────
        if self.rotate_deg:
            frame = self._rotate(frame)

        # ── 缩放填充全屏 ─────────────────────────────
        scr_w, scr_h = 1024, 600
        fh, fw = frame.shape[:2]
        scale = max(scr_w / fw, scr_h / fh)
        nw, nh = int(fw * scale), int(fh * scale)
        frame = cv2.resize(frame, (nw, nh))
        sx = (nw - scr_w) // 2
        sy = (nh - scr_h) // 2
        frame = frame[sy:sy+scr_h, sx:sx+scr_w]
        h, w = frame.shape[:2]

        # ── 状态条 ───────────────────────────────────
        status = 'NO PERSON'
        color = (0, 0, 255)
        if targets is not None and targets.targets:
            t0 = targets.targets[0]
            status = f'TRACK #{t0.track_id} | {targets.fps}FPS'
            color = (0, 255, 0)
        cv2.putText(frame, status, (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        cv2.imshow(self._window, frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = DisplayNode()
    node.sub_img = node.create_subscription(CompressedImage, '/image', node.img_cb, 10)
    node.sub_det = node.create_subscription(PerceptionTargets, '/hobot_mono2d_body_detection', node.det_cb, 10)
    node.timer = node.create_timer(0.05, node.render)
    rclpy.spin(node)
    rclpy.shutdown()
