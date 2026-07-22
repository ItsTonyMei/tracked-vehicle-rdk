#!/usr/bin/env python3
"""
motion_arbiter — 运动仲裁节点 (/cmd_vel 唯一发布者)

职责:
  1. CI1302 V6 语音识别 -> 运动命令 + FOLLOW/STOP + 锁/解锁 relay
  2. FOLLOW 模式: /locked_target(Point: dist+y+vx) 覆写速度
     - LiDAR 距离 -> 连续速度映射 (Schmitt 迟滞后退 + EKF vx 前馈)
     - LiDAR 侧向 -> PD 转向 (k_p=0.4, k_d=1.2, ±5cm deadband)
     - fallback: body_tracking angular.z
  3. 急停: /emergency_stop -> 立即发布零速
  4. /cmd_vel 唯一发布者, 消除多写冲突
  5. /system_ready 信号后欢迎语 + 手势锁/解锁 CI1302 语音确认

状态机:
  VOICE_MANUAL -> 语音运动命令 10Hz 重发 (3s 窗口)
  FOLLOWING   -> 20Hz 独立定时器驱动跟随速度 (不依赖 body_track)
                 LiDAR 优先 0.3s staleness, body_track fallback
                 "停止"/"关闭跟随" -> VOICE_MANUAL

跟随参数:
  dist_min=0.7m, dist_near=1.2m, dist_far=3.0m
  back_enter=0.85m, back_exit=1.0m (迟滞), back_vel_floor=-0.15
  vel_fast=0.8, vel_slow=0.2, vel_back=-0.3
  k_angular=0.4, k_angular_d=1.2, deadband=0.05m, lpf_alpha=0.25
  k_ff_approach=1.2 (EKF vx 前馈增益)

数据流:
  感知权威  -> /locked_target (Point: x=dist, y=lat, z=EKF_vx) + /emergency_stop
  跟踪策略  -> /cmd_vel_body_track (Twist, angular fallback)
  语音输入  -> CI1302 UART (A5 FA V6 协议: 0x04/0x05 锁/解)
  手势反馈  -> /voice_gesture_cmd (Int32, relay 至 perception_node)
  唯一输出  -> /cmd_vel (Twist, motor_bridge 消费)

协议 V6 (V01843 SDK): A5 FA 00 [TYPE] [CMD_ID] 00 [CKSUM] FB (8 bytes)
  TYPE=0x81 CI1302->Host, TYPE=0x82 Host->CI1302
  0x04=锁定跟随者, 0x05=解除跟随者 (V6 新增)
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


class State(enum.IntEnum):
    VOICE_MANUAL = 0
    FOLLOWING = 1


class MotionArbiter(Node):

    CMD_MAP = {
        0x04: ('LOCK_TARGET',   None),   # V6: 锁定跟随者 (gesture→voice feedback)
        0x05: ('RELEASE_TARGET', None),  # V6: 解除跟随者 (gesture→voice feedback)
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

        # ── 横向 PD 控制参数 ──
        self._k_angular = self.declare_parameter('k_angular', 0.4).value
        self._k_angular_d = self.declare_parameter('k_angular_damping', 1.2).value
        self._angular_deadband = self.declare_parameter('angular_deadband_m', 0.05).value
        self._angular_lpf = self.declare_parameter('angular_lpf_alpha', 0.25).value

        # ── 后退迟滞 + EKF 前馈参数 ──
        self._back_enter_m = self.declare_parameter('back_enter_m', 0.85).value
        self._back_exit_m = self.declare_parameter('back_exit_m', 1.0).value
        self._back_vel_floor = self.declare_parameter('back_vel_floor', -0.15).value
        self._k_ff_approach = self.declare_parameter('k_ff_approach', 1.2).value

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

        # ── PD 横向控制状态 ──
        self._prev_y = 0.0
        self._prev_y_dot = 0.0
        self._prev_angular_z = 0.0

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
        self._emergency_stop = False

        try:
            self._ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Voice module opened: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open voice port {port}: {e}')
            raise

        time.sleep(0.8)
        self._ser.flushInput()

        self._welcome_played = False

        self._timer = self.create_timer(0.2, self._poll)         # CI1302 串口轮询
        self._action_timer = self.create_timer(0.1, self._publish_action)  # 语音动作独立重发 10Hz
        self._follow_timer = self.create_timer(0.05, self._follow_timer_cb)  # 跟随速度 20Hz (不依赖 body_track)
        self._follow_pub.publish(Bool(data=False))
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
        self._write_cmd(0x02)
        self.get_logger().info('Welcome triggered — ALL SYSTEMS GO')

    # ═════════════════════════════════════════════════════════════
    # 串口

    def _write_cmd(self, cmd_id):
        cksum = (0xA5 + 0xFA + 0x00 + self._TYPE_TO_CI1302 + cmd_id + 0x00) & 0xFF
        frame = bytes([0xA5, 0xFA, 0x00, self._TYPE_TO_CI1302, cmd_id, 0x00, cksum, self._TAIL])
        try:
            self._ser.write(frame)
        except (serial.SerialException, OSError):
            self._try_serial_reconnect()

    def _close_serial(self):
        if hasattr(self, '_ser') and self._ser.is_open:
            self._ser.close()

    def _try_serial_reconnect(self):
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

        # ── CI1302 语音反馈: FOLLOWING 模式下手势锁/解锁 → 播报确认 ──
        if self._state != State.FOLLOWING or new_id == prev_id:
            return
        if new_id is not None:
            # 锁定 / 切换目标 → 播报 "锁定跟随者"
            self._write_cmd(0x04)
            tag = f'#{new_id}'
            if prev_id is not None:
                tag = f'#{prev_id}→#{new_id}'
            self.get_logger().info(f'CI1302: lock feedback → {tag}')
        else:
            # 解除锁定 → 播报 "解除跟随者"
            self._write_cmd(0x05)
            self.get_logger().info(f'CI1302: release feedback ← #{prev_id}')

    def _on_emergency_stop(self, msg: Bool):
        if msg.data and not self._emergency_stop:
            self.get_logger().warn('EMERGENCY STOP: detected - zero vel NOW')
            self._publish_vel('E-STOP', self._STOP_VEL)
            self._last_cmd_id = None
            self._last_cmd_vel = None
        self._emergency_stop = msg.data

    def _distance_to_linear_vel(self, dist_m):
        """LiDAR 融合距离 → 线速度 (连续映射 + EKF 前馈 + 后退迟滞).

        返回 None 表示无可用距离, 调用方应回退到 bbox 判定.

        后退迟滞: 进入后退 < back_enter_m, 退出 > back_exit_m (Schmitt trigger).
        EKF 前馈: 人在靠近 (vx<0) → 提前增加后退量, 补偿 LiDAR 延迟.
        速度地板: 后退不低于 back_vel_floor, 克服履带车静摩擦."""
        if dist_m is None or not math.isfinite(dist_m) or dist_m <= 0:
            return None

        # ── 后退区 (迟滞 + 前馈 + 地板) ──
        in_back_zone = dist_m < self._back_enter_m
        if in_back_zone or (self._was_backing and dist_m < self._back_exit_m):
            self._was_backing = True
            # 0.5m → 全速后退, _back_enter_m → 速度地板 (graduated)
            if dist_m < 0.5:
                vel = float(self._vel_back)
            else:
                ratio = (dist_m - 0.5) / (self._back_enter_m - 0.5)
                vel = self._vel_back * (1.0 - ratio)  # -0.3→~0
            # 地板: 不低于 _back_vel_floor (-0.15), 克服静摩擦
            vel = min(vel, self._back_vel_floor)
            # EKF 前馈: 人在靠近时增加后退量
            if math.isfinite(self._locked_vx) and self._locked_vx < -0.1:
                vel += self._k_ff_approach * self._locked_vx  # vx<0 → vel 更负
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
            self._pub.publish(out)
            return

        if self._emergency_stop:
            self._pub.publish(out)  # 纯零速, 禁止旋转
            return

        bt_msg = self._body_track_msg
        bt_fresh = (bt_msg is not None and
                    (now - self._body_track_ts) < 0.3)
        lidar_fresh = (math.isfinite(self._locked_dist) and
                       (now - self._locked_dist_ts) < 0.3)

        # ── 角速度: PD 控制 (LiDAR 侧向偏移优先, body_track fallback) ──
        if lidar_fresh and math.isfinite(self._locked_y):
            y = self._locked_y
            # 死区: |y| < 5cm → 不修正
            if abs(y) < self._angular_deadband:
                raw_z = 0.0
            else:
                # 数值微分 + 低通滤波
                dt = max(now - self._locked_dist_ts, 0.01)
                raw_dot = (y - self._prev_y) / dt
                y_dot = (self._angular_lpf * raw_dot +
                         (1.0 - self._angular_lpf) * self._prev_y_dot)
                self._prev_y_dot = y_dot
                # PD: angular = -(k_p * y + k_d * y_dot)
                raw_z = -(self._k_angular * y + self._k_angular_d * y_dot)
            # 输出低通滤波 (平滑 10Hz →
            out.angular.z = (self._angular_lpf * raw_z +
                             (1.0 - self._angular_lpf) * self._prev_angular_z)
            self._prev_y = y
            self._prev_angular_z = out.angular.z
        elif bt_fresh:
            out.angular = bt_msg.angular

        # ── 线速度: LiDAR 距离映射 (含 EKF 前馈 + 后退迟滞) ──
        if lidar_fresh:
            vel = self._distance_to_linear_vel(self._locked_dist)
            if vel is not None:
                out.linear.x = float(vel)
            elif bt_fresh:
                out.linear = bt_msg.linear
        elif bt_fresh:
            out.linear = bt_msg.linear

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
    # 语音命令分发

    def _on_voice(self, cmd_id):
        if cmd_id not in self.CMD_MAP:
            self.get_logger().info(f'UNMAPPED voice ID=0x{cmd_id:02X}')
            return

        name, vel = self.CMD_MAP[cmd_id]
        now = self.get_clock().now().nanoseconds / 1e9

        # 防 CI1302 扬声器→麦克风反馈误触发: 命令后 500ms 冷却
        if now - self._last_voice_ts < 0.5:
            return
        self._last_voice_ts = now

        if cmd_id == 0x04:  # LOCK_TARGET (V6: 语音"锁定跟随者" → 锁定最近行人)
            self._voice_gesture_pub.publish(Int32(data=1))
            self.get_logger().info('VOICE: LOCK_TARGET → relay to perception')
            return

        if cmd_id == 0x05:  # RELEASE_TARGET (V6: 语音"解除跟随者" → 解除锁定)
            self._voice_gesture_pub.publish(Int32(data=0))
            self.get_logger().info('VOICE: RELEASE_TARGET → relay to perception')
            return

        if cmd_id == 0x0D:  # FOLLOW_ON
            if self._state != State.FOLLOWING:
                self._state = State.FOLLOWING
                self._follow_pub.publish(Bool(data=True))
                self._last_cmd_id = None
                self._last_cmd_vel = None
                self.get_logger().info('VOICE: FOLLOW_ON → FOLLOWING mode')
            return

        if cmd_id == 0x0E:  # FOLLOW_OFF
            self._exit_following('VOICE: FOLLOW_OFF')
            return

        if name == 'STOP':
            if self._state == State.FOLLOWING:
                self._exit_following('VOICE: STOP (exit follow)')
            else:
                self._publish_vel('STOP', self._STOP_VEL)
                self._last_cmd_id = None
                self._last_cmd_vel = None
            return

        self._publish_vel(name, vel)
        self._last_cmd_ts = now
        self._last_cmd_id = cmd_id
        self._last_cmd_vel = vel
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
