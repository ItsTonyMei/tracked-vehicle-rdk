#!/usr/bin/env python3
"""本地屏显 — 零拷贝 NV12 直读 + 手势闪框 + 多尺度距离 + 渲染线程分离"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from hbm_img_msgs.msg import HbmMsg1080P
from ai_msgs.msg import PerceptionTargets
import cv2
import numpy as np
import threading
import time


class DisplayNode(Node):
    def __init__(self):
        super().__init__('display_node')
        self.bbox_ref = self.declare_parameter('bbox_ref_width', 500.0).value
        self.bbox_ref_dist = self.declare_parameter('bbox_ref_dist', 2.0).value

        self._lock = threading.Lock()
        self._frame = None       # BGR frame, ready to render
        self._targets = None
        self._locked_id = None
        self._gesture_ts = 0.0
        self._gesture_votes = {}
        self._VOTE_THRESHOLD = 30
        self._flash = 0          # flash timer: 0=off, >0=countdown
        self._flash_color = (0, 255, 0)
        self._running = True

        # Subscriptions
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.sub_nv12 = self.create_subscription(HbmMsg1080P, '/hbmem_img', self.nv12_cb, qos)
        self.sub_det = self.create_subscription(PerceptionTargets, '/hobot_mono2d_body_detection', self.det_cb, 10)
        self.sub_ges = self.create_subscription(PerceptionTargets, '/hobot_hand_gesture_detection', self.gesture_cb, 10)

        # Render thread
        self._render_thread = threading.Thread(target=self._render_loop, daemon=True)
        self._render_thread.start()
        self.get_logger().info('display_node OK (NV12 zero-copy)')

    # ── Callbacks (fast, no cv2 calls) ─────────────────
    def nv12_cb(self, msg: HbmMsg1080P):
        h, w = msg.height, msg.width
        nv12 = np.frombuffer(msg.data, dtype=np.uint8, count=h*w*3//2).reshape(h+h//2, w)
        bgr = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
        with self._lock:
            self._frame = bgr

    def det_cb(self, msg: PerceptionTargets):
        with self._lock:
            self._targets = msg

    def gesture_cb(self, msg: PerceptionTargets):
        now = time.time()
        for t in msg.targets:
            for attr in t.attributes:
                try:
                    code = int(attr.value)
                except (ValueError, TypeError):
                    continue
                if code == 0:
                    for k in list(self._gesture_votes.keys()):
                        self._gesture_votes[k] = max(0, self._gesture_votes[k] - 2)
                    continue
                self._gesture_votes[code] = self._gesture_votes.get(code, 0) + 1
                for k in list(self._gesture_votes.keys()):
                    if k != code:
                        self._gesture_votes[k] = 0
                if self._gesture_votes.get(code, 0) >= self._VOTE_THRESHOLD:
                    self._gesture_votes.clear()
                    if now - self._gesture_ts < 3.0:
                        return
                    if code == 11:
                        self._on_ok()
                    elif code == 5:
                        self._on_palm()
                return

    def _on_ok(self):
        with self._lock:
            targets = self._targets
        if not targets:
            return
        best, best_area = None, 0
        for t in targets.targets:
            if t.type != 'person': continue
            for roi in t.rois:
                if roi.type == 'body':
                    area = roi.rect.width * roi.rect.height
                    if area > best_area:
                        best_area = area
                        best = t
        if best:
            self._locked_id = best.track_id
            self._gesture_ts = time.time()
            self._flash = 15       # 15 frames green flash
            self._flash_color = (0, 255, 0)
            self.get_logger().info(f'LOCKED track_id={best.track_id}')

    def _on_palm(self):
        if self._locked_id is None:
            return
        self._locked_id = None
        self._gesture_ts = time.time()
        self._flash = 15           # 15 frames red flash
        self._flash_color = (0, 0, 255)
        self.get_logger().info('UNLOCKED')

    # ── Render loop (separate thread) ──────────────────
    def _render_loop(self):
        cv2.namedWindow('RDK X5 Tracker', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('RDK X5 Tracker', 1024, 600)
        cv2.moveWindow('RDK X5 Tracker', 0, 0)
        cv2.setWindowProperty('RDK X5 Tracker', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        flash_frame = 0
        while self._running:
            with self._lock:
                frame = self._frame.copy() if self._frame is not None else None
                targets = self._targets
                locked_id = self._locked_id
                flash = self._flash
                flash_color = self._flash_color
                if flash > 0:
                    self._flash -= 1

            if frame is None:
                time.sleep(0.05)
                continue

            orig_h, orig_w = frame.shape[:2]
            # Draw detections
            if targets:
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

                    # P3: Multi-scale distance (bbox width + kps shoulder)
                    dist = self._estimate_dist(t, r)

                    is_locked = (t.track_id == locked_id)
                    color = (0, 0, 255) if is_locked else (0, 255, 0)
                    thickness = 3 if is_locked else 2
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
                    cv2.rectangle(frame, (x1, y1 - 24), (x1 + 200, y1), color, -1)
                    cv2.putText(frame, f'#{t.track_id} {dist:.1f}m', (x1 + 4, y1 - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    bx, by = x1 + r.width//2, y1 + r.height//2
                    cv2.line(frame, (bx, by), (orig_w//2, orig_h//2), (255, 0, 255), 1)

            # Crosshair
            cx0, cy0 = orig_w//2, orig_h//2
            cv2.line(frame, (cx0-20, cy0), (cx0+20, cy0), (128, 128, 128), 1)
            cv2.line(frame, (cx0, cy0-20), (cx0, cy0+20), (128, 128, 128), 1)
            cv2.ellipse(frame, (cx0, cy0), (40, 40), 0, 0, 360, (100, 100, 100), 1)

            # P1: Flash border on gesture lock/unlock
            if flash > 0:
                cv2.rectangle(frame, (0, 0), (orig_w-1, orig_h-1), flash_color, max(2, flash//3))

            # Scale to screen
            scr_w, scr_h = 1024, 600
            fh, fw = frame.shape[:2]
            scale = max(scr_w/fw, scr_h/fh)
            nw, nh = int(fw*scale), int(fh*scale)
            frame = cv2.resize(frame, (nw, nh))
            sx, sy = (nw-scr_w)//2, (nh-scr_h)//2
            frame = frame[sy:sy+scr_h, sx:sx+scr_w]

            # Status bar
            if locked_id:
                status = f'LOCKED #{locked_id}'
                sc = (0, 0, 255)
            elif targets and targets.targets:
                status = f'DET #{targets.targets[0].track_id} | {targets.fps}FPS'
                sc = (0, 255, 255)
            else:
                status = 'OK=lock  Palm=unlock'
                sc = (0, 0, 255)
            cv2.putText(frame, status, (10, frame.shape[0]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, sc, 2)

            cv2.imshow('RDK X5 Tracker', frame)
            cv2.waitKey(1)
            time.sleep(0.08)  # ~12fps

        cv2.destroyAllWindows()

    # ── Multi-scale distance estimation ────────────────
    def _estimate_dist(self, target, body_rect):
        """bbox 宽度 + body_kps 肩宽融合"""
        # Method 1: bbox width (fast, always available)
        if body_rect.width > 0:
            d_bbox = (self.bbox_ref * self.bbox_ref_dist) / body_rect.width
        else:
            d_bbox = 2.0

        # Method 2: body_kps shoulder width (more accurate if available)
        kps = None
        for pt in target.points:
            if pt.type == 'body_kps' and len(pt.point) >= 11:
                # shoulders: kps[5]=left, kps[6]=right
                lx, ly = pt.point[5].x, pt.point[5].y
                rx, ry = pt.point[6].x, pt.point[6].y
                kps = abs(rx - lx)
                break

        if kps and kps > 10:  # valid shoulder width in px
            # Average shoulder width ~0.42m, calibrated at 500px=2m
            d_kps = (500 * 2.0 * 0.42) / (kps * 0.5)  # simplified calibration
            return d_kps * 0.6 + d_bbox * 0.4  # weighted fusion
        return d_bbox

    def shutdown(self):
        self._running = False
        if self._render_thread.is_alive():
            self._render_thread.join(timeout=1.0)


def main():
    rclpy.init()
    node = DisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
