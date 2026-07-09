#!/usr/bin/env python3
"""
motion_arbiter — 运动仲裁节点 (/cmd_vel 唯一发布者)

职责:
  1. CI1302 语音识别 → 运动命令 + FOLLOW/STOP 状态切换
  2. FOLLOW 模式: 由 /locked_target LiDAR 融合距离覆写 linear.x，
     保留 body_tracking 的 angular.z (bbox 居中旋转可靠)
  3. /cmd_vel 唯一发布者，消除多写冲突
  4. 收到 /system_ready 信号后触发欢迎语播报

状态机:
  VOICE_MANUAL → 语音运动命令直接发布 /cmd_vel，3s 超时自动 STOP
  FOLLOWING   → 订阅 /locked_target 覆写线速度，
                 /cmd_vel_body_track 提供角速度
                 "停止"/"关闭跟随" → VOICE_MANUAL

数据源:
  感知权威 → perception_node (/locked_target, /locked_track_id)
  跟踪策略 → body_tracking (/cmd_vel_body_track, angular 居中)
  语音输入 → CI1302 UART (A5 FA 协议)
  唯一输出 → /cmd_vel (Twist, 串口桥接消费)

协议 V01843: A5 FA 00 [TYPE] [CMD_ID] 00 [CKSUM] FB (8 bytes)
  TYPE=0x81 CI1302→Host 识别结果, TYPE=0x82 Host→CI1302 触发播报
  CKSUM = (A5+FA+00+TYPE+CMD+00) & 0xFF
  固件: CI1302_chinese_1mic_V01843, USE_SEPARATE_WAKEUP_EN=1
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, Int32
import serial
import time
import math
import enum


class State(enum.IntEnum):
    VOICE_MANUAL = 0
    FOLLOWING = 1


class MotionArbiter(Node):

    CMD_MAP = {
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
        self._dist_far = self.declare_parameter('follow_dist_far_m', 2.5).value
        self._dist_near = self.declare_parameter('follow_dist_near_m', 1.2).value
        self._dist_min = self.declare_parameter('follow_dist_min_m', 0.7).value
        self._vel_fast = self.declare_parameter('follow_vel_fast', 0.3).value
        self._vel_slow = self.declare_parameter('follow_vel_slow', 0.1).value
        self._vel_back = self.declare_parameter('follow_vel_back', -0.2).value

        self._state = State.VOICE_MANUAL
        self._last_cmd_ts = 0.0
        self._last_cmd_id = None
        self._body_track_msg = None

        # ── LiDAR 锁目标距离 (来自 perception_node) ──
        self._locked_dist = float('nan')
        self._locked_track_id = -1

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._follow_pub = self.create_publisher(Bool, '/follow_active', 10)
        self._sub_bt = self.create_subscription(
            Twist, '/cmd_vel_body_track', self._on_body_track, 10)
        self._sub_ready = self.create_subscription(
            Bool, '/system_ready', self._on_system_ready, 10)
        self._sub_target = self.create_subscription(
            Float32, '/locked_target', self._on_locked_target, 10)
        self._sub_locked_id = self.create_subscription(
            Int32, '/locked_track_id', self._on_locked_track_id, 10)

        try:
            self._ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Voice module opened: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open voice port {port}: {e}')
            raise

        time.sleep(0.8)
        self._ser.flushInput()

        self._welcome_played = False

        self._timer = self.create_timer(0.1, self._poll)
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
        except serial.SerialException:
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
        except serial.SerialException as e:
            self.get_logger().warn(f'Voice serial reconnect failed: {e}')

    # ═════════════════════════════════════════════════════════════
    # LiDAR 距离覆写

    def _on_locked_target(self, msg: Float32):
        self._locked_dist = msg.data

    def _on_locked_track_id(self, msg: Int32):
        self._locked_track_id = msg.data

    def _distance_to_linear_vel(self, dist_m):
        """LiDAR 融合距离 → 线速度映射。

        返回 None 表示无可用距离, 调用方应回退到 bbox 判定."""
        if dist_m is None or not math.isfinite(dist_m) or dist_m <= 0:
            return None
        if dist_m < self._dist_min:
            return self._vel_back   # 太近 → 后退
        if dist_m < self._dist_near:
            return 0.0              # 合适范围 → 停止
        if dist_m < self._dist_far:
            # 线性插值: dist_near→0, dist_far→vel_slow
            ratio = (dist_m - self._dist_near) / (self._dist_far - self._dist_near)
            return self._vel_slow * ratio
        return self._vel_fast       # 远 → 全速前进

    # ═════════════════════════════════════════════════════════════
    # body_tracking 中继 (角速度保留, 线速度由 LiDAR 覆写)

    def _on_body_track(self, msg: Twist):
        self._body_track_msg = msg
        if self._state == State.FOLLOWING and self._last_cmd_id is None:
            self._publish_following_vel()

    def _publish_following_vel(self):
        """FOLLOWING 模式运动发布: LiDAR 距离覆写 linear.x, bbox 角速度保留."""
        if self._body_track_msg is None:
            return

        out = Twist()
        out.angular = self._body_track_msg.angular  # bbox 居中旋转可靠

        vel = self._distance_to_linear_vel(self._locked_dist)
        if vel is not None:
            out.linear.x = float(vel)
            self.get_logger().debug(
                f'FOLLOW LiDAR: dist={self._locked_dist:.2f}m → linear.x={vel:.2f}')
        else:
            # 回退: 原样透传 body_tracking (bbox 判定)
            out.linear = self._body_track_msg.linear
            self.get_logger().debug(
                'FOLLOW fallback: LiDAR unavailable, using bbox velocity',
                throttle_duration_sec=3.0)

        self._pub.publish(out)

    # ═════════════════════════════════════════════════════════════
    # 轮询

    def _poll(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if self._last_cmd_id is not None and now - self._last_cmd_ts > self._action_duration:
            if self._state == State.FOLLOWING:
                self.get_logger().info('Voice motion done, resuming follow relay')
                self._publish_following_vel()
            else:
                self._publish_vel('AUTO_STOP', self._STOP_VEL)
            self._last_cmd_id = None

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
        except serial.SerialException:
            self._try_serial_reconnect()

    # ═════════════════════════════════════════════════════════════
    # 语音命令分发

    def _on_voice(self, cmd_id):
        if cmd_id not in self.CMD_MAP:
            self.get_logger().info(f'UNMAPPED voice ID=0x{cmd_id:02X}')
            return

        name, vel = self.CMD_MAP[cmd_id]
        now = self.get_clock().now().nanoseconds / 1e9

        if cmd_id == 0x0D:  # FOLLOW_ON
            if self._state != State.FOLLOWING:
                self._state = State.FOLLOWING
                self._follow_pub.publish(Bool(data=True))
                self._last_cmd_id = None
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
            return

        self._publish_vel(name, vel)
        self._last_cmd_ts = now
        self._last_cmd_id = cmd_id
        self.get_logger().info(
            f'VOICE: {name} (ID=0x{cmd_id:02X}) '
            f'[{self._state.name}]')

    def _exit_following(self, log_msg):
        self._state = State.VOICE_MANUAL
        self._follow_pub.publish(Bool(data=False))
        self._publish_vel('STOP', self._STOP_VEL)
        self._last_cmd_id = None
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
