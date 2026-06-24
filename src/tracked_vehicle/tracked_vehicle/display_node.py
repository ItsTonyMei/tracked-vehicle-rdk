#!/usr/bin/env python3
"""本地屏显 — 手势锁定跟随 (v2: 空间匹配 + 状态机)"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from ai_msgs.msg import PerceptionTargets
import cv2
import numpy as np


class DisplayNode(Node):
    """本地屏显 — 手势锁定跟随.

    所有回调与 render 运行在默认 SingleThreadedExecutor 上，无需显式同步。
    若切换为 MultiThreadedExecutor，需为 _frame/_targets/_locked_id 加锁。

    锁定状态机:
      IDLE     — 无锁定, 等待 OK 手势
      LOCKED   — 已锁定某人, 其他人进出不影响; 另一人做 OK 则切换; 被锁者做 Palm 则解除
      HOLDING  — 被锁者短暂消失 (<2s), 维持锁定等待重现; 超时 → IDLE
    """

    def __init__(self):
        super().__init__('display_node')
        self.bbox_ref = self.declare_parameter('bbox_ref_width', 500.0).value
        self.bbox_ref_dist = self.declare_parameter('bbox_ref_dist', 2.0).value
        self.rotate_deg = self.declare_parameter('rotate_deg', 0).value

        # ── 可配置参数 ──
        self._VOTE_THRESHOLD = self.declare_parameter('gesture_vote_threshold', 30).value
        self._ok_cooldown_s = self.declare_parameter('ok_cooldown_s', 3.0).value
        self._lost_hold_s = self.declare_parameter('lost_hold_s', 2.0).value
        self._empty_reset_s = self.declare_parameter('empty_reset_s', 10.0).value
        self._max_det_age_s = self.declare_parameter('max_det_age_s', 0.5).value

        # ── 帧数据 ──
        self._frame = None
        self._targets = None
        self._last_det_ts = 0.0

        # ── 手势投票 ──
        self._gesture_ts = 0.0
        self._gesture_votes = {}

        # ── 锁定状态机 ──
        self._locked_id = None    # 当前锁定的 track_id (None=IDLE)
        self._lost_since = 0.0    # 被锁者消失的时间戳 (0=可见)
        self._empty_since = 0.0   # 画面无人开始的时间戳 (0=有人)

        # ── 渲染 ──
        self._flash = 0
        self._flash_color = (0, 255, 0)
        self._window = 'RDK X5 Tracker'
        self._init_display()

        # QoS: 相机帧用 BEST_EFFORT 避免背压; AI 检测用 BEST_EFFORT 深度 5
        qos_img = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        qos_det = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub_img = self.create_subscription(CompressedImage, '/image', self.img_cb, qos_img)
        self.sub_det = self.create_subscription(
            PerceptionTargets, '/hobot_mono2d_body_detection', self.det_cb, qos_det)
        self.sub_ges = self.create_subscription(
            PerceptionTargets, '/hobot_hand_gesture_detection', self.gesture_cb, qos_det)
        self.timer = self.create_timer(0.1, self.render)

    # ═══════════════════════════════════════════════════════════════
    # 显示初始化
    # ═══════════════════════════════════════════════════════════════

    def _init_display(self):
        cv2.namedWindow(self._window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._window, 1024, 600)
        cv2.moveWindow(self._window, 0, 0)
        cv2.setWindowProperty(self._window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        self.get_logger().info('display_node OK')

    # ═══════════════════════════════════════════════════════════════
    # 空间匹配工具
    # ═══════════════════════════════════════════════════════════════

    @staticmethod
    def _find_body_roi(target):
        for roi in target.rois:
            if roi.type == 'body':
                return roi.rect
        return None

    @staticmethod
    def _point_in_rect(px, py, rect):
        return (rect.x_offset <= px <= rect.x_offset + rect.width
                and rect.y_offset <= py <= rect.y_offset + rect.height)

    def _match_gesture_to_person(self, gesture_msg):
        """手势-人体空间匹配: 返回做 OK 手势的 body track_id, 或 None."""
        if self._targets is None or gesture_msg is None:
            return None

        # 收集所有 body ROI
        body_map = {}  # track_id → rect
        for bt in self._targets.targets:
            if bt.type != 'person':
                continue
            r = self._find_body_roi(bt)
            if r is not None:
                body_map[bt.track_id] = r

        if not body_map:
            return None

        # 对手势消息中每个 target，取其第一个 ROI 中心点进行匹配
        for gt in gesture_msg.targets:
            for groi in gt.rois:
                gx = groi.rect.x_offset + groi.rect.width / 2.0
                gy = groi.rect.y_offset + groi.rect.height / 2.0
                for tid, brect in body_map.items():
                    if self._point_in_rect(gx, gy, brect):
                        return tid
        return None

    # ═══════════════════════════════════════════════════════════════
    # 回调
    # ═══════════════════════════════════════════════════════════════

    def img_cb(self, msg: CompressedImage):
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        self._frame = cv2.imdecode(raw, cv2.IMREAD_COLOR)

    def det_cb(self, msg: PerceptionTargets):
        now = self.get_clock().now().nanoseconds / 1e9
        self._targets = msg
        self._last_det_ts = now

        # ── 画面无人检测 → 超时重置 ──
        has_people = any(t.type == 'person' for t in msg.targets)
        if not has_people:
            if self._empty_since == 0.0:
                self._empty_since = now
            elif now - self._empty_since > self._empty_reset_s:
                if self._locked_id is not None:
                    self.get_logger().info(
                        f'画面无人 {self._empty_reset_s:.0f}s, 自动解除 #{self._locked_id}')
                self._locked_id = None
                self._lost_since = 0.0
                self._empty_since = 0.0
        else:
            self._empty_since = 0.0

        # ── 追踪被锁者存在性 ──
        if self._locked_id is not None:
            still_visible = any(
                t.track_id == self._locked_id and t.type == 'person'
                for t in msg.targets
            )
            if not still_visible:
                if self._lost_since == 0.0:
                    self._lost_since = now
                elif now - self._lost_since > self._lost_hold_s:
                    self.get_logger().info(
                        f'#{self._locked_id} 消失 >{self._lost_hold_s:.0f}s, 解除锁定')
                    self._locked_id = None
                    self._lost_since = 0.0
            else:
                self._lost_since = 0.0

    def gesture_cb(self, msg: PerceptionTargets):
        """手势回调: 投票防抖 + 手势-人体空间匹配."""
        now = self.get_clock().now().nanoseconds / 1e9
        triggered = None
        triggered_msg = None

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
                    triggered = code
                    triggered_msg = msg
                    break
            if triggered:
                break

        if triggered:
            self._gesture_votes.clear()
            if now - self._gesture_ts < self._ok_cooldown_s:
                return
            if triggered == 11:
                self._on_ok(now, triggered_msg)
            elif triggered == 5:
                self._on_palm(now)

    # ═══════════════════════════════════════════════════════════════
    # 锁定状态机
    # ═══════════════════════════════════════════════════════════════

    def _on_ok(self, now, gesture_msg):
        """OK 手势: 空间匹配 → 锁定做出手势的人."""
        if self._targets is None:
            return
        if now - self._last_det_ts > self._max_det_age_s:
            self.get_logger().warn('检测数据过期，忽略锁定')
            return

        matched_id = self._match_gesture_to_person(gesture_msg)
        if matched_id is None:
            self.get_logger().warn('OK 手势未能匹配到任何人')
            return

        if matched_id == self._locked_id:
            return  # 同一人，无需操作

        old_id = self._locked_id
        self._locked_id = matched_id
        self._gesture_ts = now
        self._lost_since = 0.0
        self._flash = 15
        self._flash_color = (0, 255, 0)

        if old_id is None:
            self.get_logger().info(f'LOCKED track_id={matched_id}')
        else:
            self.get_logger().info(f'SWITCHED #{old_id} -> #{matched_id}')

    def _on_palm(self, now):
        """Palm 手势: 仅当已锁定时解除."""
        if self._locked_id is None:
            return
        self.get_logger().info(f'UNLOCKED (was #{self._locked_id})')
        self._locked_id = None
        self._lost_since = 0.0
        self._gesture_ts = now
        self._flash = 15
        self._flash_color = (0, 0, 255)

    # ═══════════════════════════════════════════════════════════════
    # 渲染
    # ═══════════════════════════════════════════════════════════════

    @staticmethod
    def _rotate(img, deg):
        if deg == 90:
            return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        elif deg in (270, -90):
            return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        elif deg == 180:
            return cv2.rotate(img, cv2.ROTATE_180)
        return img

    def render(self):
        if self._frame is None:
            return
        frame = self._frame.copy()
        orig_h, orig_w = frame.shape[:2]

        targets = self._targets
        holding = (self._locked_id is not None and self._lost_since > 0.0)

        if targets is not None:
            for t in targets.targets:
                if t.type != 'person':
                    continue
                body_roi = self._find_body_roi(t)
                if body_roi is None:
                    continue

                r = body_roi
                x1, y1 = int(r.x_offset), int(r.y_offset)
                x2, y2 = x1 + int(r.width), y1 + int(r.height)
                dist = (self.bbox_ref * self.bbox_ref_dist) / r.width if r.width > 0 else 0

                is_locked = (t.track_id == self._locked_id)
                if is_locked:
                    box_color = (0, 0, 255) if not holding else (0, 165, 255)
                    thickness = 3
                else:
                    box_color = (0, 255, 0)
                    thickness = 2
                cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, thickness)

                label = f'#{t.track_id} {dist:.1f}m'
                if is_locked and holding:
                    label += ' ?'
                cv2.rectangle(frame, (x1, y1 - 24), (x1 + 180, y1), box_color, -1)
                cv2.putText(frame, label, (x1 + 4, y1 - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                bx = x1 + int(r.width) // 2
                by = y1 + int(r.height) // 2
                cv2.line(frame, (bx, by), (orig_w // 2, orig_h // 2), (255, 0, 255), 1)

        # 中心十字
        cx0, cy0 = orig_w // 2, orig_h // 2
        cv2.line(frame, (cx0 - 20, cy0), (cx0 + 20, cy0), (128, 128, 128), 1)
        cv2.line(frame, (cx0, cy0 - 20), (cx0, cy0 + 20), (128, 128, 128), 1)
        cv2.ellipse(frame, (cx0, cy0), (40, 40), 0, 0, 360, (100, 100, 100), 1)

        if self.rotate_deg:
            frame = self._rotate(frame, self.rotate_deg)

        # 缩放
        scr_w, scr_h = 1024, 600
        fh, fw = frame.shape[:2]
        scale = max(scr_w / fw, scr_h / fh)
        nw, nh = int(fw * scale), int(fh * scale)
        frame = cv2.resize(frame, (nw, nh))
        sx, sy = (nw - scr_w) // 2, (nh - scr_h) // 2
        frame = frame[sy:sy + scr_h, sx:sx + scr_w]
        h = frame.shape[0]

        # 手势闪框
        if self._flash > 0:
            t = max(1, self._flash // 3)
            cv2.rectangle(frame, (0, 0), (scr_w - 1, scr_h - 1), self._flash_color, t)
            self._flash -= 1

        # 状态条
        if holding:
            remaining = max(0, self._lost_hold_s - (
                self.get_clock().now().nanoseconds / 1e9 - self._lost_since))
            status = f'HOLDING #{self._locked_id} {remaining:.1f}s | Palm to release'
            color = (0, 165, 255)
        elif self._locked_id:
            status = f'LOCKED #{self._locked_id} | OK to switch, Palm to release'
            color = (0, 0, 255)
        elif targets and any(t.type == 'person' for t in targets.targets):
            pids = [str(t.track_id) for t in targets.targets if t.type == 'person']
            status = f'DETECT [{",".join(pids)}] {targets.fps}FPS | OK=lock Palm=unlock'
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
    rclpy.spin(DisplayNode())
    rclpy.shutdown()
