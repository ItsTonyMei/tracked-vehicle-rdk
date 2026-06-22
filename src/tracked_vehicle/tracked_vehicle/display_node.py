#!/usr/bin/env python3
"""本地屏显 — 手势锁定跟随"""
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
        self._locked_id = None    # 手势锁定的 track_id
        self._gesture = ''         # 当前手势
        self._gesture_ts = 0.0     # 手势时间戳
        self._gesture_votes = {}   # 手势投票 {code: count}
        self._VOTE_THRESHOLD = 30  # 连续30帧相同手势才触发 (~0.5s)
        self._window = 'RDK X5 Tracker'
        self._init_display()

    def _init_display(self):
        cv2.namedWindow(self._window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._window, 1024, 600)
        cv2.moveWindow(self._window, 0, 0)
        cv2.setWindowProperty(self._window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        self.get_logger().info('display_node OK')

    def img_cb(self, msg: CompressedImage):
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        self._frame = cv2.imdecode(raw, cv2.IMREAD_COLOR)

    def det_cb(self, msg: PerceptionTargets):
        self._targets = msg

    def gesture_cb(self, msg: PerceptionTargets):
        """手势回调: 投票防抖, OK(14)=锁定, Palm(5)=解除"""
        now = self.get_clock().now().nanoseconds / 1e9
        for t in msg.targets:
            for attr in t.attributes:
                try:
                    code = int(attr.value)
                except:
                    continue
                if code == 0:
                    continue  # 无手势, 重置投票
                self._gesture_votes[code] = self._gesture_votes.get(code, 0) + 1
                # 重置其他手势计数
                for k in list(self._gesture_votes.keys()):
                    if k != code:
                        self._gesture_votes[k] = 0
                # 达到阈值触发
                if self._gesture_votes.get(code, 0) >= self._VOTE_THRESHOLD:
                    self._gesture_votes.clear()
                    if now - self._gesture_ts < 3.0:  # 3s 冷却
                        return
                    if code == 14:  # OK
                        self._on_ok(now)
                    elif code == 5:  # Palm
                        self._on_palm(now)
                return  # 只处理第一个有效手势

    def _on_ok(self, now):
        """锁定画面中面积最大的人"""
        if self._targets is None:
            return
        best = None
        best_area = 0
        for t in self._targets.targets:
            if t.type != 'person':
                continue
            for roi in t.rois:
                if roi.type == 'body':
                    area = roi.rect.width * roi.rect.height
                    if area > best_area:
                        best_area = area
                        best = t
        if best:
            self._locked_id = best.track_id
            self._gesture = 'OK'
            self._gesture_ts = now
            self.get_logger().info(f'LOCKED track_id={best.track_id}')

    def _on_palm(self, now):
        if self._locked_id is None:
            return  # 未锁定时忽略 Palm
        self._locked_id = None
        self._gesture = 'PALM'
        self._gesture_ts = now
        self.get_logger().info('UNLOCKED')

    def _rotate(self, img):
        if self.rotate_deg == 90:
            return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotate_deg in (270, -90):
            return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        elif self.rotate_deg == 180:
            return cv2.rotate(img, cv2.ROTATE_180)
        return img

    def render(self):
        if self._frame is None:
            return
        frame = self._frame.copy()
        orig_h, orig_w = frame.shape[:2]

        targets = self._targets
        if targets is not None:
            for t in targets.targets:
                if t.type != 'person':
                    continue
                body_roi = None
                for roi in t.rois:
                    if roi.type == 'body':
                        body_roi = roi.rect
                        break
                if body_roi is None:
                    continue

                r = body_roi
                x1, y1 = int(r.x_offset), int(r.y_offset)
                x2, y2 = x1 + int(r.width), y1 + int(r.height)
                dist = (self.bbox_ref * self.bbox_ref_dist) / r.width if r.width > 0 else 0

                # 框颜色: 锁定=红(粗), 未锁定=绿
                is_locked = (t.track_id == self._locked_id)
                box_color = (0, 0, 255) if is_locked else (0, 255, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 3 if is_locked else 2)

                label = f'#{t.track_id} {dist:.1f}m'
                cv2.rectangle(frame, (x1, y1 - 24), (x1 + 160, y1), box_color, -1)
                cv2.putText(frame, label, (x1 + 4, y1 - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                bx, by = x1 + int(r.width)//2, y1 + int(r.height)//2
                cv2.line(frame, (bx, by), (orig_w//2, orig_h//2), (255, 0, 255), 1)

        # 中心十字
        cx0, cy0 = orig_w // 2, orig_h // 2
        cv2.line(frame, (cx0 - 20, cy0), (cx0 + 20, cy0), (128, 128, 128), 1)
        cv2.line(frame, (cx0, cy0 - 20), (cx0, cy0 + 20), (128, 128, 128), 1)
        cv2.ellipse(frame, (cx0, cy0), (40, 40), 0, 0, 360, (100, 100, 100), 1)

        if self.rotate_deg:
            frame = self._rotate(frame)

        # 缩放
        scr_w, scr_h = 1024, 600
        fh, fw = frame.shape[:2]
        scale = max(scr_w / fw, scr_h / fh)
        nw, nh = int(fw * scale), int(fh * scale)
        frame = cv2.resize(frame, (nw, nh))
        sx, sy = (nw - scr_w)//2, (nh - scr_h)//2
        frame = frame[sy:sy+scr_h, sx:sx+scr_w]
        h, w = frame.shape[:2]

        # 状态条
        if self._locked_id:
            status = f'LOCKED #{self._locked_id} | OK to lock new, Palm to release'
            color = (0, 0, 255)
        elif targets and targets.targets:
            status = f'DETECT #{targets.targets[0].track_id} | {targets.fps}FPS | OK=lock Palm=unlock'
            color = (0, 255, 255)
        else:
            status = 'WAITING | OK gesture to lock'
            color = (0, 0, 255)
        cv2.putText(frame, status, (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        cv2.imshow(self._window, frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = DisplayNode()
    node.sub_img = node.create_subscription(CompressedImage, '/image', node.img_cb, 10)
    node.sub_det = node.create_subscription(PerceptionTargets, '/hobot_mono2d_body_detection', node.det_cb, 10)
    node.sub_ges = node.create_subscription(PerceptionTargets, '/hobot_hand_gesture_detection', node.gesture_cb, 10)
    node.timer = node.create_timer(0.05, node.render)
    rclpy.spin(node)
    rclpy.shutdown()
