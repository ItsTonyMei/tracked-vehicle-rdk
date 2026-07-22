#!/usr/bin/env python3
"""感知节点 — LiDAR-Camera 融合 + 手势锁定 + 障碍物急停 + HDMI 渲染

数据职责 (单一权威源):
  - /locked_target     (Point): x=距离, y=侧向偏移, z=EKF vx 逼近速度 (前馈用)
  - /locked_track_id   (Int32): 当前锁定的 track_id, 解锁时为 -1
  - /emergency_stop    (Bool):  前方 0.5m / +-15deg 有障碍物 (被锁人豁免)
  - /system_ready      (Bool):  系统启动就绪信号
  - JPEG 按需解码 (img_cb 60fps仅存原始字节, render 15Hz时解码)
  - fusion.update 单次调用 (去重, 15Hz predict + 10Hz 完整管线)
  - HDMI: 检测框 + LiDAR 融合距离 + 系统状态栏 + 手势投票进度条

传感器:
  Camera: GS130W SC132GS, 72deg HFOV @ 960x544, f=1.75mm 广角, 60fps via mono2d
  LiDAR:  YDLidar T-mini Plus, 360deg @ 10Hz, 430pts, 0.84deg/pt, 胸高度 ~150cm

融合管线: 自适应聚类 -> 躯干几何过滤 -> 匈牙利角度匹配 -> EKF(x,y,vx,vy)

手势锁定 (v0.9.0 滑动窗口 + 多码并行):
  /hobot_hand_gesture_detection 属性码 OK=11 + Victory=2 并行锁定, Palm=5 解锁
  30帧窗口, ≥15命中触发, 容忍短暂掉帧; 置信度门控 (默认 0.0 即禁用)
  空间匹配: 严格 hand-in-body-rect → fallback 最近人体 (250px)
  自适应发现: 新出现手势码自动打印 GESTURE DISCOVERY 日志
  IDLE — 无锁定, OK/Victory 手势触发锁定
  LOCKED — 已锁定, 另一人 OK/Victory 则切换, Palm 则解除
  HOLDING — 被锁者短暂消失: <1s 保持原锁不 RE-ID, 1-5s 尝试 RE-ID (80px), >5s 解锁
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, LaserScan
from ai_msgs.msg import PerceptionTargets
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Point
import cv2
import numpy as np
import math

from .lidar_fusion import FusionEngine, _find_body_roi


class PerceptionNode(Node):
    """感知权威: 融合距离 + 手势锁定 + 系统状态 + HDMI 渲染."""

    def __init__(self):
        super().__init__('perception_node')
        self.rotate_deg = self.declare_parameter('rotate_deg', 0).value

        # ── 可配置参数 ──
        self._VOTE_THRESHOLD = self.declare_parameter('gesture_vote_threshold', 15).value
        self._ok_cooldown_s = self.declare_parameter('ok_cooldown_s', 3.0).value
        self._lost_hold_s = self.declare_parameter('lost_hold_s', 5.0).value
        self._lost_reid_min_s = self.declare_parameter('lost_reid_min_s', 1.0).value
        self._reid_max_dist_px = self.declare_parameter('reid_max_dist_px', 80.0).value
        self._empty_reset_s = self.declare_parameter('empty_reset_s', 10.0).value
        self._gesture_min_score = self.declare_parameter('gesture_min_score', 0.0).value
        self._gesture_match_max_px = self.declare_parameter('gesture_match_max_px', 250.0).value
        self._max_det_age_s = self.declare_parameter('max_det_age_s', 0.5).value
        self._cam_hfov_deg = self.declare_parameter('cam_hfov_deg', 72.0).value  # SC132GS rotation=90 → 72°

        # ── 帧数据 ──
        self._frame_jpeg = None   # 压缩 JPEG 原始字节 (render 按需解码, 省 CPU)
        self._targets = None
        self._last_det_ts = 0.0

        # ── 启动状态 ──
        self._startup_done = False
        self._last_startup_check = 0.0
        self._SUBSYSTEMS = [
            ('/image',             'Camera'),
            ('/hobot_mono2d_body_detection', 'Body Det'),
            ('/hobot_hand_lmk_detection',    'Hand LMK'),
            ('/hobot_hand_gesture_detection','Gesture'),
            ('/cmd_vel_body_track', 'Tracking'),
            ('/cmd_vel',           'Cmd Vel'),
            ('/follow_active',     'Voice'),
            ('/scan',              'LiDAR'),
        ]
        self._startup_ok = {}
        self._startup_order = []
        self._startup_failed = set()
        self._startup_timeout_s = 25.0

        # ── 手势投票 (滑动窗口, 多码并行) ──
        self._gesture_ts = 0.0
        self._gesture_window = []           # deque-like: list of (code, score)
        self._gesture_window_max = self.declare_parameter('gesture_window_max', 30).value
        # 锁定码列表: OK=11 + Victory=2 (✌️) 并行触发
        self._lock_codes = self.declare_parameter('lock_codes', [11, 2]).value
        self._unlock_codes = self.declare_parameter('unlock_codes', [5]).value
        # 诊断: 自动发现未识别的手势码 (仅首次打印)
        self._gesture_codes_seen = set()

        # ── 锁定状态机 ──
        self._locked_id = None
        self._lost_since = 0.0
        self._empty_since = 0.0
        self._last_known_cx = None
        self._last_known_cy = None

        # ── 渲染 ──
        self._flash = 0
        self._flash_color = (0, 255, 0)
        self._window = 'RDK X5 Tracker'
        self._init_display()

        # QoS
        qos_img = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        qos_det = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub_img = self.create_subscription(CompressedImage, '/image', self.img_cb, qos_img)
        self.sub_det = self.create_subscription(
            PerceptionTargets, '/hobot_mono2d_body_detection', self.det_cb, qos_det)
        self.sub_ges = self.create_subscription(
            PerceptionTargets, '/hobot_hand_gesture_detection', self.gesture_cb, qos_det)
        self.sub_follow = self.create_subscription(
            Bool, '/follow_active', self.follow_cb, 10)
        self._follow_active = False
        self._sub_voice_gesture = self.create_subscription(
            Int32, '/voice_gesture_cmd', self._on_voice_gesture_cmd, 10)  # V6: CI1302→gesture relay
        qos_scan = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_scan)
        self._scan = None
        self._fusion = FusionEngine(cam_hfov_deg=self._cam_hfov_deg)
        self._fused = {}

        # ── 下行发布 (运动仲裁者消费) ──
        self._locked_target_pub = self.create_publisher(Point, '/locked_target', 10)
        self._locked_track_id_pub = self.create_publisher(Int32, '/locked_track_id', 10)
        self._emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self._last_published_id = -2

        self._ready_pub = self.create_publisher(Bool, '/system_ready', 10)
        self.timer = self.create_timer(1.0/15.0, self.render)
        self._start_ts = self.get_clock().now().nanoseconds / 1e9

    # ═══════════════════════════════════════════════════════════════
    # 显示初始化

    def _init_display(self):
        cv2.namedWindow(self._window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._window, 1024, 600)
        cv2.moveWindow(self._window, 0, 0)
        cv2.setWindowProperty(self._window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        self.get_logger().info('perception_node OK')

    # ═══════════════════════════════════════════════════════════════
    # 启动进度检测

    def _check_startup_progress(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_startup_check < 2.0:
            return
        self._last_startup_check = now

        for topic, name in self._SUBSYSTEMS:
            if not self._startup_ok.get(name, False):
                pubs = self.get_publishers_info_by_topic(topic)
                if pubs:
                    self._startup_ok[name] = True
                    self._startup_order.append(name)
                    self.get_logger().info(
                        f'Startup [{len(self._startup_order)}/{len(self._SUBSYSTEMS)}]: {name}')

        for topic, name in self._SUBSYSTEMS:
            if (not self._startup_ok.get(name, False) and
                now - self._start_ts > self._startup_timeout_s):
                self._startup_failed.add(name)
                self._startup_ok[name] = False
                self.get_logger().warn(f'Startup TIMEOUT: {name} ({topic})')

    # ═══════════════════════════════════════════════════════════════
    # 空间工具

    def _estimate_bbox_distance(self, rect, img_w):
        if rect.width <= 0:
            return 0.0
        aspect = rect.width / max(rect.height, 1)
        if aspect > 2.0:
            visible_h = 1.5
        elif aspect > 1.2:
            visible_h = 1.0
        else:
            visible_h = 0.7
        f_px = (img_w / 2.0) / math.tan(math.radians(self._cam_hfov_deg / 2.0))
        return f_px * visible_h / rect.width

    @staticmethod
    def _point_in_rect(px, py, rect):
        return (rect.x_offset <= px <= rect.x_offset + rect.width
                and rect.y_offset <= py <= rect.y_offset + rect.height)

    @staticmethod
    def _collect_body_ids(targets):
        if targets is None:
            return []
        ids = []
        seen = set()
        for t in targets.targets:
            if t.track_id not in seen and _find_body_roi(t) is not None:
                ids.append(str(t.track_id))
                seen.add(t.track_id)
        return ids

    # ═══════════════════════════════════════════════════════════════
    # 手势-人体空间匹配

    def _match_gesture_to_person(self, gesture_msg):
        if self._targets is None or gesture_msg is None:
            return None
        body_map = {}
        for bt in self._targets.targets:
            if bt.type != 'person':
                continue
            r = _find_body_roi(bt)
            if r is not None:
                body_map[bt.track_id] = r
        if not body_map:
            return None
        # Pass 1: 严格手在人体 bbox 内
        for gt in gesture_msg.targets:
            for groi in gt.rois:
                if groi.type != 'hand':
                    continue
                gx = groi.rect.x_offset + groi.rect.width / 2.0
                gy = groi.rect.y_offset + groi.rect.height / 2.0
                for tid, brect in body_map.items():
                    if self._point_in_rect(gx, gy, brect):
                        return tid
        # Pass 2 (fallback): 伸臂场景 — 找最近人体 (< _gesture_match_max_px px)
        best_id, best_dist = None, float('inf')
        for gt in gesture_msg.targets:
            for groi in gt.rois:
                if groi.type != 'hand':
                    continue
                gx = groi.rect.x_offset + groi.rect.width / 2.0
                gy = groi.rect.y_offset + groi.rect.height / 2.0
                for tid, brect in body_map.items():
                    bx = brect.x_offset + brect.width / 2.0
                    by = brect.y_offset + brect.height / 2.0
                    d = ((gx - bx) ** 2 + (gy - by) ** 2) ** 0.5
                    if d < best_dist:
                        best_dist = d
                        best_id = tid
        if best_dist < self._gesture_match_max_px:
            return best_id
        return None

    # ═══════════════════════════════════════════════════════════════
    # 回调

    def img_cb(self, msg: CompressedImage):
        # 只存原始字节不解码: /image 60fps, 渲染 15Hz → 解码量降为 1/4
        self._frame_jpeg = msg.data

    @staticmethod
    def _has_body(targets):
        if targets is None:
            return False
        for t in targets.targets:
            for roi in t.rois:
                if roi.type == 'body':
                    return True
        return False

    @staticmethod
    def _target_visible(targets, track_id):
        if targets is None:
            return False
        for t in targets.targets:
            if t.track_id == track_id:
                for roi in t.rois:
                    if roi.type == 'body':
                        return True
        return False

    def _get_body_center(self, targets, track_id):
        if targets is None:
            return None
        for t in targets.targets:
            if t.track_id == track_id:
                for roi in t.rois:
                    if roi.type == 'body':
                        return (roi.rect.x_offset + roi.rect.width / 2.0,
                                roi.rect.y_offset + roi.rect.height / 2.0)
        return None

    def _find_nearest_body(self, targets, cx, cy, max_dist):
        best_id, best_dist = None, float('inf')
        if targets is None:
            return None, best_dist
        for t in targets.targets:
            for roi in t.rois:
                if roi.type == 'body':
                    bx = roi.rect.x_offset + roi.rect.width / 2.0
                    by = roi.rect.y_offset + roi.rect.height / 2.0
                    d = ((bx - cx) ** 2 + (by - cy) ** 2) ** 0.5
                    if d < best_dist:
                        best_dist = d
                        best_id = t.track_id
        return (best_id, best_dist) if best_dist < max_dist else (None, best_dist)

    def det_cb(self, msg: PerceptionTargets):
        now = self.get_clock().now().nanoseconds / 1e9
        self._targets = msg
        self._last_det_ts = now

        if not self._has_body(msg):
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

        if self._locked_id is not None:
            if self._target_visible(msg, self._locked_id):
                self._lost_since = 0.0
                c = self._get_body_center(msg, self._locked_id)
                if c:
                    self._last_known_cx, self._last_known_cy = c
            else:
                if self._lost_since == 0.0:
                    self._lost_since = now
                # HOLDING: 等待 _lost_reid_min_s 后才尝试 RE-ID,
                # 防止单帧丢失即把锁切换到旁边的另一个人
                if now - self._lost_since >= self._lost_reid_min_s:
                    if self._last_known_cx is not None:
                        matched_id, dist = self._find_nearest_body(
                            msg, self._last_known_cx, self._last_known_cy,
                            self._reid_max_dist_px)
                        if matched_id is not None:
                            self.get_logger().info(
                                f'RE-ID #{self._locked_id} -> #{matched_id} (dist={dist:.0f}px)')
                            self._locked_id = matched_id
                            self._lost_since = 0.0
                if self._lost_since > 0.0 and now - self._lost_since > self._lost_hold_s:
                    self.get_logger().info(
                        f'#{self._locked_id} 消失 >{self._lost_hold_s:.0f}s, 解除锁定')
                    self._locked_id = None
                    self._lost_since = 0.0

    def gesture_cb(self, msg: PerceptionTargets):
        """滑动窗口手势投票: 多码并行 + 置信度门控 + 自适应发现.

        OK=11, Victory=2(✌️) 可并行触发锁定; Palm=5 触发解锁.
        30 帧窗口内 ≥15 帧命中即触发, 容忍短暂掉帧.
        首次出现的新手势码自动打印, 方便发现 Victory 等新码."""
        now = self.get_clock().now().nanoseconds / 1e9

        for t in msg.targets:
            for attr in t.attributes:
                try:
                    code = int(attr.value)
                    score = float(attr.confidence)
                except (ValueError, TypeError):
                    continue

                # 自适应发现: 打印未见过的手势码
                if code != 0 and code not in self._gesture_codes_seen:
                    self._gesture_codes_seen.add(code)
                    self.get_logger().warn(
                        f'GESTURE DISCOVERY: code={code} score={score:.3f} '
                        f'(lock_codes={self._lock_codes}, unlock_codes={self._unlock_codes})')

                # 置信度门控: 低置信度不计入窗口
                if code != 0 and score < self._gesture_min_score:
                    continue

                # 滑动窗口
                self._gesture_window.append((code, score))
                if len(self._gesture_window) > self._gesture_window_max:
                    self._gesture_window.pop(0)

                # 检查锁定触发 (多码并行)
                for lock_code in self._lock_codes:
                    lock_hits = sum(1 for c, _ in self._gesture_window
                                    if c == lock_code)
                    if lock_hits >= self._VOTE_THRESHOLD:
                        if now - self._gesture_ts >= self._ok_cooldown_s:
                            self._gesture_window.clear()
                            self.get_logger().info(
                                f'GESTURE LOCK: code={lock_code} hits={lock_hits}')
                            self._on_ok(now, msg)
                            return

                # 检查解锁触发
                for unlock_code in self._unlock_codes:
                    unlock_hits = sum(1 for c, _ in self._gesture_window
                                      if c == unlock_code)
                    if unlock_hits >= self._VOTE_THRESHOLD:
                        if now - self._gesture_ts >= self._ok_cooldown_s:
                            self._gesture_window.clear()
                            self.get_logger().info(
                                f'GESTURE UNLOCK: code={unlock_code} hits={unlock_hits}')
                            self._on_palm(now)
                            return

    def follow_cb(self, msg: Bool):
        self._follow_active = msg.data

    def scan_cb(self, msg: LaserScan):
        self._scan = msg

    # ═══════════════════════════════════════════════════════════════
    # 锁定状态机

    def _on_ok(self, now, gesture_msg):
        """OK 手势 → 空间匹配 → 锁定."""
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
            return
        old_id = self._locked_id
        self._locked_id = matched_id
        self._gesture_ts = now
        self._lost_since = 0.0
        self._flash = 15
        self._flash_color = (0, 0, 255)
        if old_id is None:
            self.get_logger().info(f'LOCKED track_id={matched_id} (OK)')
        else:
            self.get_logger().info(f'SWITCHED #{old_id} -> #{matched_id} (OK)')

    def _on_palm(self, now):
        """Palm 手势 → 解除锁定."""
        if self._locked_id is None:
            return
        self.get_logger().info(f'UNLOCKED #{self._locked_id} (Palm)')
        self._locked_id = None
        self._lost_since = 0.0
        self._gesture_ts = now
        self._flash = 15
        self._flash_color = (0, 255, 0)

    # ═══════════════════════════════════════════════════════════════
    # V6: CI1302 语音命令 → 手势等效操作

    def _on_voice_gesture_cmd(self, msg: Int32):
        """CI1302 语音"锁定跟随者"/"解除跟随者" → 等效手势操作.
        data=1: 锁定最近行人 (等效 OK 手势)
        data=0: 解除当前锁定 (等效 Palm 手势)"""
        now = self.get_clock().now().nanoseconds / 1e9
        if msg.data == 0:  # Unlock
            if self._locked_id is not None:
                self.get_logger().info(f'VOICE GESTURE: unlock #{self._locked_id}')
                self._locked_id = None
                self._lost_since = 0.0
                self._gesture_ts = now
                self._flash = 15
                self._flash_color = (0, 255, 0)
        elif msg.data == 1:  # Lock nearest person
            if self._targets is None or not self._has_body(self._targets):
                self.get_logger().warn('VOICE GESTURE: no person detected to lock')
                return
            # 锁定画面中面积最大 (最近) 的行人
            best_id = None
            best_area = 0
            for t in self._targets.targets:
                if t.type != 'person':
                    continue
                r = _find_body_roi(t)
                if r is None:
                    continue
                area = r.width * r.height
                if area > best_area:
                    best_area = area
                    best_id = t.track_id
            if best_id is None:
                self.get_logger().warn('VOICE GESTURE: no valid body ROI found')
                return
            old_id = self._locked_id
            if best_id == old_id:
                self.get_logger().info(f'VOICE GESTURE: already locked #{best_id}')
                return
            self._locked_id = best_id
            self._lost_since = 0.0
            self._gesture_ts = now
            self._flash = 15
            self._flash_color = (0, 0, 255)
            if old_id is None:
                self.get_logger().info(f'VOICE GESTURE: lock #{best_id}')
            else:
                self.get_logger().info(f'VOICE GESTURE: switch #{old_id}→#{best_id}')

    # ═══════════════════════════════════════════════════════════════
    # 系统状态读取

    _FONT = cv2.FONT_HERSHEY_SIMPLEX
    _FONT_SCALE = 0.7
    _FONT_THICK = 2
    _LABEL_H = 32
    _DOT_R = 7

    _prev_cpu_total = 0
    _prev_cpu_idle = 0

    @staticmethod
    def _read_sys_info():
        info = {}
        try:
            with open('/sys/class/thermal/thermal_zone0/temp') as f:
                info['temp'] = float(f.read().strip()) / 1000.0
        except Exception:
            info['temp'] = 0.0
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
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._sys_info_ts > 1.0:
            self._sys_info = self._read_sys_info()
            self._sys_info_ts = now
            try:
                with open('/proc/stat') as f:
                    fields = f.readline().split()[1:]
                jiffies = [int(x) for x in fields]
                total = sum(jiffies)
                idle = jiffies[3] + (jiffies[4] if len(jiffies) > 4 else 0)
                if self._prev_cpu_total > 0:
                    d_total = total - self._prev_cpu_total
                    d_idle = idle - self._prev_cpu_idle
                    self._cpu_pct = (d_total - d_idle) / d_total * 100.0 if d_total > 0 else 0.0
                self._prev_cpu_total = total
                self._prev_cpu_idle = idle
            except Exception:
                self._cpu_pct = 0.0
        return self._sys_info

    # ═══════════════════════════════════════════════════════════════
    # 渲染

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
        jpeg = self._frame_jpeg
        if jpeg is None:
            return

        # ── JPEG 解码 ──
        raw = np.frombuffer(jpeg, dtype=np.uint8)
        frame = cv2.imdecode(raw, cv2.IMREAD_COLOR)
        if frame is None:
            return
        orig_h, orig_w = frame.shape[:2]

        # ── LiDAR-Camera 融合 (单次调用, 15Hz predict, 10Hz 完整管线) ──
        targets = self._targets
        now = self.get_clock().now().nanoseconds / 1e9
        self._fused = self._fusion.update(
            self._scan, targets, orig_w, now)
        holding = (self._locked_id is not None and self._lost_since > 0.0)

        # ── 发布锁定目标距离 + 侧向偏移 + EKF逼近速度 + track_id ──
        if self._locked_id is not None and self._locked_id in self._fused:
            fd = self._fused[self._locked_id]
            # EKF 速度前馈: vx<0 表示人在靠近, 供 motion_arbiter 预判后退
            ekf = self._fusion._tracks.get(self._locked_id)
            ekf_vx = float(ekf.state['vx']) if ekf else 0.0
            self._locked_target_pub.publish(
                Point(x=float(fd['dist']), y=float(fd.get('y', 0.0)), z=ekf_vx))
        else:
            self._locked_target_pub.publish(Point(x=float('nan'), y=0.0, z=0.0))

        if self._locked_id != self._last_published_id:
            self._locked_track_id_pub.publish(
                Int32(data=self._locked_id if self._locked_id is not None else -1))
            self._last_published_id = self._locked_id

        # ── 障碍物紧急停止检测 ──
        emergency = False
        # 被锁人的角度 (用于豁免: 靠近的锁目标不应触发急停)
        locked_angle = None
        if (self._locked_id is not None and
            self._locked_id in self._fused):
            fd_locked = self._fused[self._locked_id]
            locked_angle = abs(math.degrees(math.atan2(
                fd_locked['y'], fd_locked['x'])))
        for tid, fd in self._fused.items():
            if tid >= 0:
                continue
            # 跳过被锁人附近的障碍物 (人在靠近 → 前进/后退中, 非障碍)
            if locked_angle is not None:
                obs_ang = abs(math.degrees(math.atan2(fd['y'], fd['x'])))
                if abs(obs_ang - locked_angle) < 15.0:
                    continue
            if fd['dist'] < 0.5:
                obs_angle = abs(math.degrees(math.atan2(fd['y'], fd['x'])))
                if obs_angle < 15.0:
                    emergency = True
                    break
        self._emergency_stop_pub.publish(Bool(data=emergency))

        if targets is not None:
            for t in targets.targets:
                if t.type != 'person':
                    continue
                body_roi = _find_body_roi(t)
                if body_roi is None:
                    continue

                r = body_roi
                x1, y1 = int(r.x_offset), int(r.y_offset)
                x2, y2 = x1 + int(r.width), y1 + int(r.height)

                fd = self._fused.get(t.track_id)
                if fd is not None:
                    dist = fd['dist']
                else:
                    dist = self._estimate_bbox_distance(r, orig_w)

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
                cv2.line(frame, (bx, by), (orig_w // 2, orig_h // 2), box_color, 2)

        # 中心十字
        cx0, cy0 = orig_w // 2, orig_h // 2
        cv2.line(frame, (cx0 - 25, cy0), (cx0 + 25, cy0), (255, 255, 255), 2)
        cv2.line(frame, (cx0, cy0 - 25), (cx0, cy0 + 25), (255, 255, 255), 2)
        cv2.ellipse(frame, (cx0, cy0), (40, 40), 0, 0, 360, (255, 255, 255), 2)

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

        # ── X5 系统状态栏 ──
        si = self._get_sys_info()
        cpu_pct = self._cpu_pct
        bpu_pct = si.get('bpu_pct', 0)
        mem_pct = si.get('mem_pct', 0)
        temp = si.get('temp', 0)

        def _draw_dot(x, y, pct, warn_th=80, crit_th=95):
            if pct >= crit_th:
                c = (0, 0, 255)
            elif pct >= warn_th:
                c = (0, 255, 255)
            else:
                c = (0, 255, 0)
            cv2.circle(frame, (x, y), self._DOT_R, c, -1)

        row1_y = 22
        x = 10
        for label, pct in [('CPU', cpu_pct), ('BPU', bpu_pct), ('MEM', mem_pct)]:
            _draw_dot(x + self._DOT_R, row1_y, pct)
            cv2.putText(frame, f'{label}:{pct:.0f}%', (x + 20, row1_y + 8),
                        self._FONT, self._FONT_SCALE, (255, 255, 255), self._FONT_THICK)
            x += 160

        if temp >= 90:
            tc = (0, 0, 255)
        elif temp >= 80:
            tc = (0, 255, 255)
        else:
            tc = (0, 255, 0)
        cv2.circle(frame, (x + self._DOT_R, row1_y), self._DOT_R, tc, -1)
        cv2.putText(frame, f'TEMP:{temp:.0f}C', (x + 20, row1_y + 8),
                    self._FONT, self._FONT_SCALE, (255, 255, 255), self._FONT_THICK)

        row2_y = 52
        fps_val = targets.fps if targets is not None else 0
        if fps_val > 30:
            fps_dot = (0, 255, 0)
        elif fps_val > 10:
            fps_dot = (0, 255, 255)
        elif fps_val > 0:
            fps_dot = (0, 0, 255)
        else:
            fps_dot = (100, 100, 100)
        fps_str = f'FPS:{fps_val:.0f}' if fps_val > 0 else 'FPS:--'
        cv2.circle(frame, (10 + self._DOT_R, row2_y), self._DOT_R, fps_dot, -1)
        cv2.putText(frame, fps_str, (30, row2_y + 8),
                    self._FONT, self._FONT_SCALE, (255, 255, 255), self._FONT_THICK)

        x = 170
        if self._follow_active:
            mode_str = 'MODE:FOLLOW'
            mode_dot = (0, 255, 0)
        else:
            mode_str = 'MODE:MANUAL'
            mode_dot = (100, 100, 100)
        cv2.circle(frame, (x + self._DOT_R, row2_y), self._DOT_R, mode_dot, -1)
        cv2.putText(frame, mode_str, (x + 20, row2_y + 8),
                    self._FONT, self._FONT_SCALE, (255, 255, 255), self._FONT_THICK)

        # ── 手势投票状态: 显示当前活跃码及命中数 ──
        gx = 370
        if self._gesture_window:
            codes = set(c for c, s in self._gesture_window if c != 0)
            best = None
            best_hits = 0
            for code in codes:
                hits = sum(1 for c, _ in self._gesture_window if c == code)
                if hits > best_hits:
                    best_hits = hits
                    best = code
            if best is not None and best_hits > 3:
                bar_w, bar_h = 80, 4
                bar_x, bar_y = gx, row2_y - 2
                ratio = min(1.0, best_hits / self._VOTE_THRESHOLD)
                fill_w = int(bar_w * ratio)
                cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h),
                             (50, 50, 50), -1)
                bar_color = (0, 255, 0) if best in self._lock_codes else (
                    (0, 200, 255) if best in self._unlock_codes else (150, 150, 150))
                cv2.rectangle(frame, (bar_x, bar_y), (bar_x + fill_w, bar_y + bar_h),
                             bar_color, -1)
                cv2.putText(frame, f'G:{best} {best_hits}/{self._VOTE_THRESHOLD}',
                           (bar_x + bar_w + 4, row2_y + 6),
                           self._FONT, 0.55, bar_color, self._FONT_THICK)

        if not self._startup_done:
            self._check_startup_progress()

        n_ok = len(self._startup_order)
        n_total = len(self._SUBSYSTEMS)
        n_fail = len(self._startup_failed)
        progress = n_ok / n_total if n_total > 0 else 0
        row_y = h - 52

        if self._startup_done:
            dot_color = (0, 255, 0)
        elif n_fail > 0:
            dot_color = (0, 0, 255)
        else:
            dot_color = (0, 255, 255)
        cv2.circle(frame, (10 + self._DOT_R, row_y - 2), self._DOT_R, dot_color, -1)

        if self._startup_done:
            if n_fail > 0:
                status_text = f'SYS OK ({n_fail} fail)'
                status_color = (0, 255, 255)
            else:
                status_text = 'ALL SYSTEMS GO'
                status_color = (0, 255, 0)
        else:
            status_text = 'STARTING'
            status_color = (255, 255, 255)
        status_x = 30
        cv2.putText(frame, status_text, (status_x, row_y + 6),
                    self._FONT, self._FONT_SCALE, status_color, self._FONT_THICK)
        status_w = cv2.getTextSize(status_text, self._FONT, self._FONT_SCALE, self._FONT_THICK)[0][0]

        bar_x = status_x + status_w + 12
        bar_w, bar_h = 160, 8
        bar_y = row_y - 2
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (50, 50, 50), -1)
        if progress > 0:
            fill_w = int(bar_w * progress)
            fill_color = (0, 255, 200) if n_fail == 0 else (0, 200, 255)
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + fill_w, bar_y + bar_h),
                          fill_color, -1)
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (100, 100, 100), 1)

        x = bar_x + bar_w + 8
        cv2.putText(frame, f'{n_ok}/{n_total}', (x, row_y + 6),
                    self._FONT, self._FONT_SCALE, (255, 255, 255), self._FONT_THICK)
        stat_w = cv2.getTextSize(f'{n_ok}/{n_total}', self._FONT, self._FONT_SCALE, self._FONT_THICK)[0][0]

        if n_fail > 0:
            failed_names = ', '.join(sorted(self._startup_failed))
            fail_text = f'TIMEOUT: {failed_names}'
            cv2.putText(frame, fail_text, (x + stat_w + 10, row_y + 6),
                        self._FONT, self._FONT_SCALE, (0, 0, 255), self._FONT_THICK)

        if not self._startup_done and now - self._start_ts > self._startup_timeout_s + 5.0:
            self._startup_done = True
            self._ready_pub.publish(Bool(data=True))
            if n_fail > 0:
                failed_names = ', '.join(sorted(self._startup_failed))
                self.get_logger().warn(f'STARTUP DONE with {n_fail} timeout(s): {failed_names}')

        # ── 底部状态条 ──
        bar_y = h - 16
        has_body_now = self._has_body(targets)
        follow_on = self._follow_active
        if holding:
            remaining = max(0, self._lost_hold_s - (
                self.get_clock().now().nanoseconds / 1e9 - self._lost_since))
            status = f'HOLDING #{self._locked_id} {remaining:.1f}s | Palm to release'
            color = (0, 165, 255)
        elif self._locked_id and follow_on:
            status = f'FOLLOW LOCKED #{self._locked_id} | OK=switch Palm=release'
            color = (0, 0, 255)
        elif self._locked_id:
            status = f'LOCKED #{self._locked_id} | Follow OFF'
            color = (100, 100, 100)
        elif follow_on and has_body_now:
            status = f'FOLLOW [{",".join(self._collect_body_ids(targets))}] | OK=lock Palm=unlock'
            color = (0, 255, 255)
        elif has_body_now:
            status = f'[VOICE MANUAL] DETECT [{",".join(self._collect_body_ids(targets))}]'
            color = (100, 100, 100)
        else:
            status = '[VOICE MANUAL] WAITING'
            color = (100, 100, 100)
        cv2.circle(frame, (self._DOT_R + 10, bar_y - 9), self._DOT_R, color, -1)
        cv2.putText(frame, status, (30, bar_y),
                    self._FONT, self._FONT_SCALE, (255, 255, 255), self._FONT_THICK)

        cv2.imshow(self._window, frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    rclpy.spin(PerceptionNode())
    rclpy.shutdown()
