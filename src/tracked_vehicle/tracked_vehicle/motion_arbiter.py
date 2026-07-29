#!/usr/bin/env python3
"""
motion_arbiter — 运动仲裁节点 (/cmd_vel 唯一发布者)

职责:
  1. CI1302 V7 ASR 输入 -> 运动命令 + FOLLOW/STOP + 锁/解锁 relay
     (模块被动播报, 仅发识别帧 TYPE=0x81, 不自播)
  2. M30 + piper-tts TTS 输出 -> 系统就绪/锁定/解锁/急停/模式切换语音反馈
  3. FOLLOW 模式: /locked_target(Point: dist+y+vx) 覆写速度
     - LiDAR 距离 -> 连续速度映射 (Schmitt 迟滞后退 + EKF vx 前馈)
     - LiDAR 侧向 -> P + LPF 转向 (k_p=0.4, ±5cm deadband, α=0.25)
     - fallback: body_tracking angular.z
  4. 急停: /emergency_stop -> 立即发布零速 + TTS 语音警告
  5. /cmd_vel 唯一发布者, 消除多写冲突
  6. /voice_cmd 订阅 -> Vosk ASR 节点 (备用, 当前 CI1302 承担 ASR)

状态机:
  VOICE_MANUAL -> 语音运动命令 10Hz 重发 (3s 窗口)
  FOLLOWING   -> 20Hz 独立定时器驱动跟随速度 (不依赖 body_track)
                 LiDAR 优先 0.3s staleness, body_track fallback
                 "停止"/"关闭跟随" -> VOICE_MANUAL

跟随参数:
  dist_min=0.7m, dist_near=1.2m, dist_far=3.0m
  back_enter=0.85m, back_exit=1.0m (迟滞), back_vel_floor=-0.15
  vel_fast=0.8, vel_slow=0.2, vel_back=-0.3
  k_angular=0.4, deadband=0.05m, lpf_alpha=0.25
  k_ff_approach=1.2 (EKF vx 前馈增益)

数据流:
  感知权威  -> /locked_target (Point: x=dist, y=lat, z=EKF_vx) + /emergency_stop
  跟踪策略  -> /cmd_vel_body_track (Twist, angular fallback)
  语音输入  -> CI1302 UART (A5 FA V7 协议, 仅接收 TYPE=0x81)
  手势反馈  -> /voice_gesture_cmd (Int32, relay 至 perception_node)
  TTS 输出  -> M30 USB 扬声器 (piper-tts 离线中文合成)
  唯一输出  -> /cmd_vel (Twist, motor_bridge 消费)

协议 V7 (V01843 SDK, 被动播报): A5 FA 00 [TYPE] [CMD_ID] 00 [CKSUM] FB (8 bytes)
  TYPE=0x81 CI1302->Host (识别), TYPE=0x82 保留但不再使用
  CKSUM = (A5+FA+00+TYPE+CMD+00) & 0xFF
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool, Int32
import serial
import time
import math
import enum

from .tts_engine import TTSEngine


class State(enum.IntEnum):
    VOICE_MANUAL = 0
    FOLLOWING = 1


class MotionArbiter(Node):

    CMD_MAP = {
        0x01: ('WAKE',          None),   # 唤醒词 — 仅 TTS 反馈
        0x04: ('LOCK_TARGET',   None),   # 锁定跟随者
        0x05: ('RELEASE_TARGET', None),  # 解除跟随者
        0x06: ('STOP',        (0.0,  0.0,  0.0)),
        0x07: ('FORWARD',     (0.5,  0.0,  0.0)),
        0x08: ('BACKWARD',    (-0.3,  0.0,  0.0)),
        0x09: ('TURN_LEFT',   (0.2,  0.0,  0.4)),
        0x0A: ('TURN_RIGHT',  (0.2,  0.0, -0.4)),
        0x0B: ('SPIN_LEFT',   (0.0,  0.0,  0.5)),
        0x0C: ('SPIN_RIGHT',  (0.0,  0.0, -0.5)),
        0x0D: ('FOLLOW_ON',   None),
        0x0E: ('FOLLOW_OFF',  None),
    }

    # 命令 → TTS 播报文本 (用户指定, 与触发词不重复)
    _CMD_TTS = {
        0x01: '瓦力在',
        0x06: '瓦力不动',
        0x07: '瓦力向前走',
        0x08: '瓦力正在倒车中',
        0x09: '瓦力正在向左转',
        0x0A: '瓦力正在向右转',
        0x0B: '瓦力正在向左旋转',
        0x0C: '瓦力正在向右旋转',
        0x0D: '瓦力正在跟随',
        0x0E: '瓦力不再跟随',
    }

    _STOP_VEL = (0.0, 0.0, 0.0)
    _FRAME_LEN = 8
    _TYPE_FROM_CI1302 = 0x81
    _TYPE_TO_CI1302   = 0x82
    _TAIL = 0xFB

    def __init__(self):
        super().__init__('motion_arbiter')

        port = self.declare_parameter('voice_port', '/dev/voice_module').value
        baud = self.declare_parameter('voice_baud', 115200).value
        self._action_duration = self.declare_parameter('action_duration_s', 3.0).value

        # ── 跟随距离参数 ──
        self._dist_far = self.declare_parameter('follow_dist_far_m', 3.0).value
        self._dist_near = self.declare_parameter('follow_dist_near_m', 1.2).value
        self._dist_min = self.declare_parameter('follow_dist_min_m', 0.7).value
        self._vel_fast = self.declare_parameter('follow_vel_fast', 0.8).value
        self._vel_slow = self.declare_parameter('follow_vel_slow', 0.2).value
        self._vel_back = self.declare_parameter('follow_vel_back', -0.3).value

        # ── 横向 P 控制参数 (跟踪场景用纯P+LPF, 不用D: D项会对抗目标移动) ──
        self._k_angular = self.declare_parameter('k_angular', 0.4).value
        self._angular_deadband = self.declare_parameter('angular_deadband_m', 0.05).value
        self._angular_lpf = self.declare_parameter('angular_lpf_alpha', 0.25).value

        # ── 后退迟滞 + EKF 前馈参数 ──
        self._back_enter_m = self.declare_parameter('back_enter_m', 0.85).value
        self._back_exit_m = self.declare_parameter('back_exit_m', 1.0).value
        self._back_vel_floor = self.declare_parameter('back_vel_floor', -0.15).value
        self._k_ff_approach = self.declare_parameter('k_ff_approach', 1.2).value
        self._linear_accel_limit = self.declare_parameter('linear_accel_limit', 0.8).value
        self._linear_lpf = self.declare_parameter('linear_lpf_alpha', 0.30).value
        self._ff_lpf = self.declare_parameter('ff_lpf_alpha', 0.20).value

        self._state = State.VOICE_MANUAL
        self._last_cmd_ts = 0.0
        self._last_cmd_id = None
        self._last_cmd_vel = None
        self._body_track_msg = None
        self._body_track_ts = 0.0

        # ── LiDAR 锁目标 (来自 perception_node, Point: x=距离, y=侧向偏移, z=EKF逼近速度) ──
        self._locked_dist = float('nan')
        self._locked_y = 0.0
        self._locked_vx = 0.0    # EKF vx: <0=人在靠近 (前馈用)
        self._locked_dist_ts = 0.0

        # ── P + LPF 横向控制状态 ──
        self._prev_angular_z = 0.0

        # ── 线速度平滑状态 ──
        self._prev_linear_x = 0.0
        self._prev_ff_vel = 0.0    # EKF 前馈 LPF 状态

        # ── 后退迟滞状态 ──
        self._was_backing = False

        # ── 锁定人物 ID (来自 perception_node, -1=无锁) ──
        self._locked_id = None

        self._last_voice_ts = 0.0  # CI1302 防抖冷却 (扬声器→麦克风反馈抑制)

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._follow_pub = self.create_publisher(Bool, '/follow_active', 10)
        self._sub_bt = self.create_subscription(
            Twist, '/cmd_vel_body_track', self._on_body_track, 10)
        self._sub_ready = self.create_subscription(
            Bool, '/system_ready', self._on_system_ready, 10)
        self._sub_target = self.create_subscription(
            Point, '/locked_target', self._on_locked_target, 10)
        self._sub_locked_id = self.create_subscription(
            Int32, '/locked_track_id', self._on_locked_track_id, 10)
        self._sub_emergency = self.create_subscription(
            Bool, '/emergency_stop', self._on_emergency_stop, 10)
        self._voice_gesture_pub = self.create_publisher(
            Int32, '/voice_gesture_cmd', 10)  # V6: voice→gesture relay
        self._sub_voice_cmd = self.create_subscription(
            Int32, '/voice_cmd', self._on_voice_cmd, 10)  # Phase 2: Vosk ASR
        self._emergency_stop = False

        self._ser = None
        try:
            self._ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Voice module opened: {port} @ {baud}')
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f'Voice module not available ({port}): {e} — running without voice')

        if self._ser is not None:
            time.sleep(0.8)
            self._ser.flushInput()

        self._welcome_played = False

        self._timer = self.create_timer(0.2, self._poll)         # CI1302 串口轮询
        self._action_timer = self.create_timer(0.1, self._publish_action)  # 语音动作独立重发 10Hz
        self._follow_timer = self.create_timer(0.05, self._follow_timer_cb)  # 跟随速度 20Hz (不依赖 body_track)
        self._follow_pub.publish(Bool(data=False))
        self._warmup_phrases = (['瓦力系统已就绪', '瓦力已锁定跟随者', '瓦力已解除跟随者']
                                + list(self._CMD_TTS.values()))
        self._warmup_done = False

        self.get_logger().info('Motion arbiter ready — VOICE_MANUAL mode')

    def destroy_node(self):
        self._close_serial()
        super().destroy_node()

    # ═════════════════════════════════════════════════════════════
    # 欢迎语

    def _on_system_ready(self, msg: Bool):
        if self._welcome_played:
            return
        self._welcome_played = True
        self.get_logger().info('Welcome triggered — ALL SYSTEMS GO')

        # 延迟 TTS 预热: 等系统稳定后后台逐条合成, 不抢启动 CPU
        if not self._warmup_done:
            self._warmup_done = True
            import threading
            def _delayed_warmup():
                time.sleep(10)  # 等 BPU 推理稳定
                TTSEngine.get().warmup(self._warmup_phrases)
                self.get_logger().info(
                    f'TTS warmup done ({len(self._warmup_phrases)} phrases)')
            t = threading.Thread(target=_delayed_warmup, daemon=True)
            t.start()

    # ═════════════════════════════════════════════════════════════
    # 串口

    def _speak_tts(self, text):
        """TTS 播报 (M30 USB 扬声器, 非阻塞后台线程)."""
        try:
            TTSEngine.get().speak(text)
        except Exception as e:
            self.get_logger().warn(f'TTS speak failed: {e}')

    def _speak_cmd_tts(self, cmd_id):
        """播报命令对应的 TTS 反馈 (如有)."""
        text = self._CMD_TTS.get(cmd_id)
        if text:
            self._speak_tts(text)

    def _write_cmd(self, cmd_id):
        if self._ser is None:
            return
        cksum = (0xA5 + 0xFA + 0x00 + self._TYPE_TO_CI1302 + cmd_id + 0x00) & 0xFF
        frame = bytes([0xA5, 0xFA, 0x00, self._TYPE_TO_CI1302, cmd_id, 0x00, cksum, self._TAIL])
        try:
            self._ser.write(frame)
        except (serial.SerialException, OSError):
            self._try_serial_reconnect()

    def _close_serial(self):
        if hasattr(self, '_ser') and self._ser is not None and self._ser.is_open:
            self._ser.close()

    def _try_serial_reconnect(self):
        if self._ser is None:
            return
        try:
            if self._ser.is_open:
                self._ser.close()
            self._ser.open()
            self._ser.flushInput()
            self.get_logger().warn('Voice serial reconnected')
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f'Voice serial reconnect failed: {e}')

    # ═════════════════════════════════════════════════════════════
    # LiDAR 距离覆写

    def _on_locked_target(self, msg: Point):
        self._locked_dist = msg.x
        self._locked_y = msg.y
        self._locked_vx = msg.z  # EKF 逼近速度: <0=人在靠近 (前馈补偿)
        self._locked_dist_ts = self.get_clock().now().nanoseconds / 1e9

    def _on_locked_track_id(self, msg: Int32):
        prev_id = self._locked_id
        new_id = msg.data if msg.data >= 0 else None
        self._locked_id = new_id
        if self._locked_id is None:
            self._locked_dist = float('nan')
            self._locked_y = 0.0
            self._locked_dist_ts = 0.0

        # ── 语音反馈: 任意模式下手势锁/解锁 → TTS + CI1302 播报确认 ──
        if new_id == prev_id:
            return
        if new_id is not None:
            # 锁定 / 切换目标
            self._speak_tts('瓦力已锁定跟随者')
            tag = f'#{new_id}'
            if prev_id is not None:
                tag = f'#{prev_id}→#{new_id}'
            self.get_logger().info(f'lock feedback → {tag}')
        else:
            # 解除锁定
            self._speak_tts('瓦力已解除跟随者')
            self.get_logger().info(f'release feedback ← #{prev_id}')

    def _on_emergency_stop(self, msg: Bool):
        if msg.data and not self._emergency_stop:
            self.get_logger().warn('EMERGENCY STOP: detected - zero vel NOW')
            self._speak_tts('急停')
            self._publish_vel('E-STOP', self._STOP_VEL)
            self._last_cmd_id = None
            self._last_cmd_vel = None
        self._emergency_stop = msg.data

    def _distance_to_linear_vel(self, dist_m):
        """LiDAR 融合距离 → 线速度 (连续映射 + EKF 前馈 + 后退迟滞).

        返回 None 表示无可用距离, 调用方应回退到 bbox 判定.

        后退迟滞: 进入后退 < back_enter_m, 退出 > back_exit_m (Schmitt trigger).
        EKF 前馈: 人在靠近 (vx<0) → 增加后退量, LPF 平滑避免突变.
        速度地板: min(vel, floor) 确保后退不低于 _back_vel_floor (~克服静摩擦)."""
        if dist_m is None or not math.isfinite(dist_m) or dist_m <= 0:
            return None

        # ── 后退区 (迟滞 + 前馈 + 地板) ──
        in_back_zone = dist_m < self._back_enter_m
        if in_back_zone or (self._was_backing and dist_m < self._back_exit_m):
            self._was_backing = True
            # 0.5m → 全速后退, _back_enter_m → 0 (graduated)
            if dist_m < 0.5:
                vel = float(self._vel_back)       # -0.3
            else:
                ratio = min(1.0, (dist_m - 0.5) / (self._back_enter_m - 0.5))
                vel = self._vel_back * (1.0 - ratio)  # -0.3→0
            # 迟滞区 (退出门控): 距离 > back_enter_m 时不产生正向速度
            vel = min(vel, 0.0)
            # 地板: min 确保速度不低于 floor (克服静摩擦, -0.15)
            vel = min(vel, self._back_vel_floor)
            # EKF 前馈: 人在靠近时增加后退量 (LPF 平滑)
            if math.isfinite(self._locked_vx) and self._locked_vx < -0.1:
                raw_ff = self._k_ff_approach * self._locked_vx  # vx<0 → ff<0
                self._prev_ff_vel = (self._ff_lpf * raw_ff +
                                     (1.0 - self._ff_lpf) * self._prev_ff_vel)
                vel += self._prev_ff_vel
            return max(vel, self._vel_back * 1.5)  # 上限: 不超 1.5x max_back
        self._was_backing = False

        # ── 停止区 (back_exit_m ~ dist_near) ──
        if dist_m < self._dist_near:              # 1.0-1.2m: 合适, 停止
            return 0.0
        # ── 前进加速区 (1.2-3.0m) ──
        if dist_m < self._dist_far:
            ratio = (dist_m - self._dist_near) / (self._dist_far - self._dist_near)
            return self._vel_slow * ratio + (self._vel_fast - self._vel_slow) * ratio * ratio
        return self._vel_fast                     # ≥ 3.0m: 全速

    # ═════════════════════════════════════════════════════════════
    # body_tracking 中继 (角速度保留, 线速度由 LiDAR 覆写)

    def _on_body_track(self, msg: Twist):
        self._body_track_msg = msg
        self._body_track_ts = self.get_clock().now().nanoseconds / 1e9

    def _follow_timer_cb(self):
        """20Hz: 跟随模式独立定时器, 不依赖 body_track 消息到达.
        防止近距相机遮挡时 body_track 停发导致车辆僵死."""
        if self._state == State.FOLLOWING and self._last_cmd_id is None:
            self._publish_following_vel()

    def _publish_following_vel(self):
        """FOLLOWING 模式运动发布: 必须有 OK 手势锁定的人才能跟随.

        未锁定时: 即使 FOLLOWING 模式激活, 也输出零速 — 车辆原地等待锁定.
        LiDAR 优先, 0.3s staleness. PD 横向控制 + EKF 前馈后退."""
        now = self.get_clock().now().nanoseconds / 1e9
        out = Twist()

        # 安全门控: 无锁定时禁止跟随, 防止跟踪未经授权的路人
        if self._locked_id is None:
            self._prev_linear_x = 0.0    # 复位平滑状态
            self._prev_ff_vel = 0.0
            self._pub.publish(out)
            return

        if self._emergency_stop:
            self._prev_linear_x = 0.0    # 急停立即复位
            self._prev_ff_vel = 0.0
            self._pub.publish(out)  # 纯零速, 禁止旋转
            return

        bt_msg = self._body_track_msg
        bt_fresh = (bt_msg is not None and
                    (now - self._body_track_ts) < 0.3)
        lidar_fresh = (math.isfinite(self._locked_dist) and
                       (now - self._locked_dist_ts) < 0.3)

        # ── 角速度: P + LPF (LiDAR 侧向偏移优先, body_track fallback) ──
        # 跟踪场景不用 D 项: 目标移动产生的 ẏ 会被 D 误解为车辆过冲而反向修正
        if lidar_fresh and math.isfinite(self._locked_y):
            y = self._locked_y
            # 死区: |y| < 5cm → 不修正
            if abs(y) < self._angular_deadband:
                raw_z = 0.0
            else:
                raw_z = -self._k_angular * y
            # 输出低通滤波 (平滑 10Hz 阶梯)
            out.angular.z = (self._angular_lpf * raw_z +
                             (1.0 - self._angular_lpf) * self._prev_angular_z)
            self._prev_angular_z = out.angular.z
        elif bt_fresh:
            out.angular = bt_msg.angular

        # ── 线速度: LiDAR 距离映射 + LPF + 加速度限制 ──
        if lidar_fresh:
            raw_vel = self._distance_to_linear_vel(self._locked_dist)
            if raw_vel is not None:
                # LPF 平滑 (与角速度一致)
                smooth_vel = (self._linear_lpf * raw_vel +
                              (1.0 - self._linear_lpf) * self._prev_linear_x)
                # 加速度限制: max change/tick = accel_limit * dt (20Hz → dt=0.05s)
                max_dv = self._linear_accel_limit * 0.05
                dv = smooth_vel - self._prev_linear_x
                if abs(dv) > max_dv:
                    dv = math.copysign(max_dv, dv)
                    smooth_vel = self._prev_linear_x + dv
                out.linear.x = float(smooth_vel)
                self._prev_linear_x = out.linear.x
            elif bt_fresh:
                out.linear = bt_msg.linear
                self._prev_linear_x = out.linear.x
        elif bt_fresh:
            out.linear = bt_msg.linear
            self._prev_linear_x = out.linear.x
        else:
            # 无数据 → 向零衰减 (平滑停车)
            decay = self._prev_linear_x * 0.5
            out.linear.x = float(decay)
            self._prev_linear_x = out.linear.x

        self._pub.publish(out)

    # ═════════════════════════════════════════════════════════════
    # 轮询

    # ═════════════════════════════════════════════════════════════
    # 语音动作独立重发 (10Hz timer, 与 CI1302 串口完全解耦)

    def _publish_action(self):
        """10Hz 重发激活的语音动作. 独立 timer, 不受 CI1302 误识别干扰."""
        if self._last_cmd_id is None or self._last_cmd_vel is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_cmd_ts > self._action_duration:
            if self._state == State.FOLLOWING:
                self.get_logger().info('Voice motion done, resuming follow relay')
                self._publish_following_vel()
            else:
                self._publish_vel('AUTO_STOP', self._STOP_VEL)
            self._last_cmd_id = None
            self._last_cmd_vel = None
            return
        if self._emergency_stop:
            self._publish_vel('AUTO_STOP', self._STOP_VEL)
            self._last_cmd_id = None
            self._last_cmd_vel = None
            self.get_logger().warn('VOICE: cancelled by emergency stop')
            return
        self._publish_vel('REPUBLISH', self._last_cmd_vel)

    # ═════════════════════════════════════════════════════════════
    # CI1302 串口轮询

    def _poll(self):
        if self._ser is None:
            return
        try:
            count = self._ser.in_waiting
            if not count:
                return
            data = self._ser.read(count)

            for i in range(len(data) - self._FRAME_LEN + 1):
                if (data[i] == 0xA5 and data[i+1] == 0xFA and
                    data[i+2] == 0x00 and data[i+3] == self._TYPE_FROM_CI1302 and
                    data[i+7] == self._TAIL):
                    calc = (data[i] + data[i+1] + data[i+2] +
                            data[i+3] + data[i+4] + data[i+5]) & 0xFF
                    if calc == data[i+6]:
                        self._on_voice(data[i+4])
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f'Voice serial error: {e}')
            self._try_serial_reconnect()

    # ═════════════════════════════════════════════════════════════
    # Phase 2: Vosk ASR 语音命令 (/voice_cmd Int32)

    def _on_voice_cmd(self, msg: Int32):
        """接收 voice_asr 发布的 CMD ID, 路由到现有 _on_voice 分发."""
        cmd_id = msg.data
        self.get_logger().info(f'VOICE ASR: cmd=0x{cmd_id:02X}')
        self._on_voice(cmd_id)

    # ═════════════════════════════════════════════════════════════
    # 语音命令分发

    def _on_voice(self, cmd_id):
        if cmd_id not in self.CMD_MAP:
            self.get_logger().info(f'UNMAPPED voice ID=0x{cmd_id:02X}')
            return

        name, vel = self.CMD_MAP[cmd_id]
        now = self.get_clock().now().nanoseconds / 1e9

        # 防 CI1302 反馈误触发: 命令后 200ms 冷却 (播报词已与触发词区分)
        if now - self._last_voice_ts < 0.2:
            return
        self._last_voice_ts = now

        if cmd_id == 0x01:  # 唤醒词 — 仅 TTS 确认
            self._speak_tts(self._CMD_TTS.get(0x01, '我在'))
            self.get_logger().info('VOICE: WAKE → TTS feedback')
            return

        if cmd_id == 0x04:  # LOCK_TARGET
            self._voice_gesture_pub.publish(Int32(data=1))
            self.get_logger().info('VOICE: LOCK_TARGET → relay to perception')
            return

        if cmd_id == 0x05:  # RELEASE_TARGET
            self._voice_gesture_pub.publish(Int32(data=0))
            self.get_logger().info('VOICE: RELEASE_TARGET → relay to perception')
            return

        if cmd_id == 0x0D:  # FOLLOW_ON
            if self._state != State.FOLLOWING:
                self._state = State.FOLLOWING
                self._follow_pub.publish(Bool(data=True))
                self._last_cmd_id = None
                self._last_cmd_vel = None
                self._speak_cmd_tts(0x0D)
                if self._locked_id is not None:
                    self.get_logger().info(
                        f'lock feedback (FOLLOW entry) → #{self._locked_id}')
                self.get_logger().info('VOICE: FOLLOW_ON → FOLLOWING mode')
            return

        if cmd_id == 0x0E:  # FOLLOW_OFF
            if self._locked_id is not None:
                self.get_logger().info(
                    f'release feedback (FOLLOW exit) ← #{self._locked_id}')
            self._speak_cmd_tts(0x0E)
            self._exit_following('VOICE: FOLLOW_OFF')
            return

        if name == 'STOP':
            if self._state == State.FOLLOWING:
                if self._locked_id is not None:
                    self.get_logger().info(
                        f'release feedback (STOP exit) ← #{self._locked_id}')
                self._exit_following('VOICE: STOP (exit follow)')
            else:
                self._publish_vel('STOP', self._STOP_VEL)
                self._last_cmd_id = None
                self._last_cmd_vel = None
            self._speak_cmd_tts(cmd_id)
            return

        self._publish_vel(name, vel)
        self._last_cmd_ts = now
        self._last_cmd_id = cmd_id
        self._last_cmd_vel = vel
        self._speak_cmd_tts(cmd_id)
        self.get_logger().info(
            f'VOICE: {name} (ID=0x{cmd_id:02X}) '
            f'[{self._state.name}]')

    def _exit_following(self, log_msg):
        self._state = State.VOICE_MANUAL
        self._follow_pub.publish(Bool(data=False))
        self._publish_vel('STOP', self._STOP_VEL)
        self._last_cmd_id = None
        self._last_cmd_vel = None
        self.get_logger().info(f'{log_msg} → VOICE_MANUAL')

    def _publish_vel(self, name, vel):
        msg = Twist()
        msg.linear.x = float(vel[0])
        msg.linear.y = float(vel[1])
        msg.angular.z = float(vel[2])
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = MotionArbiter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
