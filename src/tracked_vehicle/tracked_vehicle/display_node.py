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

        # 收集 body detection 中的 body ROI
        body_map = {}  # track_id → rect
        for bt in self._targets.targets:
            if bt.type != 'person':
                continue
            r = self._find_body_roi(bt)
            if r is not None:
                body_map[bt.track_id] = r
        if not body_map:
            return None

        # 对手势消息中每个 target, 只取 hand 类型 ROI 的中心点进行匹配
        for gt in gesture_msg.targets:
            for groi in gt.rois:
                if groi.type != 'hand':
                    continue  # 跳过 body/head/face, 只匹配手部 ROI
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
        """手势回调: 投票防抖, OK(11)=锁定, Palm(5)=解除.
        处理第一个非零手势后立即返回，避免后续 gesture=0 降级已积累的投票."""
        now = self.get_clock().now().nanoseconds / 1e9

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
                    if now - self._gesture_ts < self._ok_cooldown_s:
                        return
                    if code == 11:
                        self._on_ok(now, msg)
                    elif code == 5:
                        self._on_palm(now)
                return  # 只处理第一个有效手势，防止后续 gesture=0 清零投票

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
        self._flash_color = (0, 0, 255)  # 锁定=红色闪框

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
        self._flash_color = (0, 255, 0)  # 解锁=绿色闪框

    # ═══════════════════════════════════════════════════════════════
    # 系统状态读取
    # ═══════════════════════════════════════════════════════════════

    _FONT = cv2.FONT_HERSHEY_SIMPLEX
    _FONT_SCALE = 0.7
    _FONT_THICK = 2
    _LABEL_H = 32
    _DOT_R = 7  # 状态指示灯半径

    # CPU 使用率需要两次采样——缓存上一次的 jiffies
    _prev_cpu_jiffies = None
    _prev_cpu_ts = 0.0

    @staticmethod
    def _read_sys_info():
        """读取 X5 系统状态: CPU%/BPU%/MEM%/温度. 每 1s 刷新一次."""
        info = {}

        # ── 温度 ──
        try:
            with open('/sys/class/thermal/thermal_zone0/temp') as f:
                info['temp'] = float(f.read().strip()) / 1000.0
        except Exception:
            info['temp'] = 0.0

        # ── 内存 % ──
        try:
            total = available = 0
            with open('/proc/meminfo') as f:
                for line in f:
                    if 'MemTotal' in line:
                        total = int(line.split()[1])
                    elif 'MemAvailable' in line:
                        available = int(line.split()[1])
                    if total and available:
                        break
            info['mem_pct'] = (total - available) / total * 100.0 if total else 0.0
        except Exception:
            info['mem_pct'] = 0.0

        # ── BPU % ──
        try:
            with open('/sys/devices/system/bpu/ratio') as f:
                info['bpu_pct'] = float(f.read().strip())
        except Exception:
            info['bpu_pct'] = 0.0

        return info

    _sys_info = {}
    _sys_info_ts = 0.0
    _cpu_pct = 0.0

    def _get_sys_info(self):
        """读取系统状态 (1Hz 缓存) + 计算 CPU 占用率."""
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._sys_info_ts > 1.0:
            self._sys_info = self._read_sys_info()
            self._sys_info_ts = now

            # CPU 占用率: 两次 /proc/stat 采样的差值
            try:
                with open('/proc/stat') as f:
                    fields = f.readline().split()[1:]  # skip "cpu"
                jiffies = [int(x) for x in fields]
                total = sum(jiffies)
                idle = jiffies[3] + (jiffies[4] if len(jiffies) > 4 else 0)  # idle + iowait
                if self._prev_cpu_jiffies is not None:
                    d_total = total - self._prev_total
                    d_idle = idle - self._prev_idle
                    self._cpu_pct = (d_total - d_idle) / d_total * 100.0 if d_total > 0 else 0.0
                self._prev_total = total
                self._prev_idle = idle
            except Exception:
                self._cpu_pct = 0.0

        return self._sys_info

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
                lw = 200 if len(label) < 12 else 260
                cv2.rectangle(frame, (x1, y1 - self._LABEL_H), (x1 + lw, y1), box_color, -1)
                cv2.putText(frame, label, (x1 + 4, y1 - 8),
                            self._FONT, self._FONT_SCALE, (255, 255, 255), self._FONT_THICK)
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

        # ── X5 系统状态栏 (左上角) ──
        si = self._get_sys_info()
        cpu_pct = self._cpu_pct
        bpu_pct = si.get('bpu_pct', 0)
        mem_pct = si.get('mem_pct', 0)
        temp = si.get('temp', 0)

        def _draw_dot(x, y, pct, warn_th=80, crit_th=95):
            """根据百分比画状态指示灯: 绿(<warn) / 黄(<crit) / 红(>=crit)"""
            if pct >= crit_th:
                c = (0, 0, 255)
            elif pct >= warn_th:
                c = (0, 255, 255)
            else:
                c = (0, 255, 0)
            cv2.circle(frame, (x, y), self._DOT_R, c, -1)

        # 第一行: 指示灯 + CPU/BPU/MEM/TEMP
        row1_y = 22
        x = 10
        for label, pct in [('CPU', cpu_pct), ('BPU', bpu_pct), ('MEM', mem_pct)]:
            _draw_dot(x + self._DOT_R, row1_y, pct)
            cv2.putText(frame, f'{label}:{pct:.0f}%', (x + 20, row1_y + 8),
                        self._FONT, self._FONT_SCALE, (220, 220, 220), self._FONT_THICK)
            x += 160

        # 温度 (特殊: >80=黄, >90=红)
        if temp >= 90:
            tc = (0, 0, 255)
        elif temp >= 80:
            tc = (0, 255, 255)
        else:
            tc = (0, 255, 0)
        cv2.circle(frame, (x + self._DOT_R, row1_y), self._DOT_R, tc, -1)
        cv2.putText(frame, f'TEMP:{temp:.0f}C', (x + 20, row1_y + 8),
                    self._FONT, self._FONT_SCALE, (220, 220, 220), self._FONT_THICK)

        # 第二行: FPS + 跟踪状态
        row2_y = 52
        fps_str = f'FPS:{targets.fps:.0f}' if targets is not None else 'FPS:--'
        cv2.putText(frame, fps_str, (10, row2_y),
                    self._FONT, self._FONT_SCALE, (180, 180, 180), self._FONT_THICK)

        # 人体检测状态指示灯
        has_people_now = targets is not None and any(
            t.type == 'person' for t in targets.targets)
        status_dot_color = (0, 255, 0) if has_people_now else (100, 100, 100)
        cv2.circle(frame, (130, row2_y - 9), self._DOT_R, status_dot_color, -1)
        people_text = 'DET' if has_people_now else '---'
        cv2.putText(frame, people_text, (148, row2_y),
                    self._FONT, self._FONT_SCALE, (180, 180, 180), self._FONT_THICK)

        # ── 状态条 (底部) ──
        bar_y = h - 16
        if holding:
            remaining = max(0, self._lost_hold_s - (
                self.get_clock().now().nanoseconds / 1e9 - self._lost_since))
            status = f'HOLDING #{self._locked_id} {remaining:.1f}s | Palm to release'
            color = (0, 165, 255)
            dot_c = (0, 165, 255)
        elif self._locked_id:
            status = f'LOCKED #{self._locked_id} | OK to switch, Palm to release'
            color = (0, 0, 255)
            dot_c = (0, 0, 255)
        elif has_people_now:
            pids = [str(t.track_id) for t in targets.targets if t.type == 'person']
            status = f'DETECT [{",".join(pids)}] | OK=lock Palm=unlock'
            color = (0, 255, 255)
            dot_c = (0, 255, 255)
        else:
            status = 'WAITING | OK gesture to lock'
            color = (0, 0, 255)
            dot_c = (100, 100, 100)
        cv2.circle(frame, (self._DOT_R + 10, bar_y - 9), self._DOT_R, dot_c, -1)
        cv2.putText(frame, status, (30, bar_y),
                    self._FONT, self._FONT_SCALE, color, self._FONT_THICK)

        cv2.imshow(self._window, frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    rclpy.spin(DisplayNode())
    rclpy.shutdown()
