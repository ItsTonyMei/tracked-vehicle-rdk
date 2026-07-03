#!/usr/bin/env python3
"""
voice_bridge — CI1302 语音识别 → /cmd_vel 仲裁节点

职责:
  1. 解析 CI1302 UART 语音识别结果 (A5 FA 00 81 [CMD] 00 [CKSUM] FB @ 115200)
  2. 状态机仲裁: VOICE_MANUAL (默认) / FOLLOWING (中继 body_tracking)
  3. 作为 /cmd_vel 唯一发布者，消除多写冲突
  4. 收到 /system_ready 信号后触发欢迎语播报 (与 display "ALL SYSTEMS GO" 同步)

状态机:
  VOICE_MANUAL → 语音运动命令直接发布 /cmd_vel，3s 超时自动 STOP
  FOLLOWING   → 中继 /cmd_vel_body_track，语音运动命令暂停 3s 后恢复
                "停止"/"关闭跟随" → VOICE_MANUAL

topic 拓扑:
  voice_bridge 订阅 /cmd_vel_body_track (body_tracking 重映射)
  voice_bridge 发布   /cmd_vel (唯一发布者)
  voice_bridge 发布   /follow_active (Bool, VOICE_MANUAL=False)

协议 V01843: A5 FA 00 [TYPE] [CMD_ID] 00 [CKSUM] FB (8 bytes)
  TYPE=0x81 CI1302→Host 识别结果, TYPE=0x82 Host→CI1302 触发播报
  CKSUM = (A5+FA+00+TYPE+CMD+00) & 0xFF
  固件: CI1302_chinese_1mic_V01843, USE_SEPARATE_WAKEUP_EN=1
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import serial
import time
import enum


class State(enum.IntEnum):
    VOICE_MANUAL = 0
    FOLLOWING = 1


class VoiceBridge(Node):

    CMD_MAP = {
        0x06: ('STOP',        (0.0,  0.0,  0.0)),   # 小车停止
        0x07: ('FORWARD',     (0.5,  0.0,  0.0)),   # 小车前进
        0x08: ('BACKWARD',    (-0.3,  0.0,  0.0)),  # 小车后退
        0x09: ('TURN_LEFT',   (0.2,  0.0,  0.4)),   # 小车左转
        0x0A: ('TURN_RIGHT',  (0.2,  0.0, -0.4)),   # 小车右转
        0x0B: ('SPIN_LEFT',   (0.0,  0.0,  0.5)),   # 小车左旋
        0x0C: ('SPIN_RIGHT',  (0.0,  0.0, -0.5)),   # 小车右旋
        0x0D: ('FOLLOW_ON',   None),                 # 开启跟随
        0x0E: ('FOLLOW_OFF',  None),                 # 关闭跟随
    }

    _STOP_VEL = (0.0, 0.0, 0.0)
    _FRAME_LEN = 8
    _TYPE_SEND = 0x81   # CI1302 → Host
    _TYPE_RECV = 0x82   # Host → CI1302
    _TAIL = 0xFB

    def __init__(self):
        super().__init__('voice_bridge')

        port = self.declare_parameter('voice_port', '/dev/voice_module').value
        baud = self.declare_parameter('voice_baud', 115200).value
        self._action_duration = self.declare_parameter('action_duration_s', 3.0).value

        self._state = State.VOICE_MANUAL
        self._last_cmd_ts = 0.0
        self._last_cmd_id = None
        self._body_track_msg = None

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._follow_pub = self.create_publisher(Bool, '/follow_active', 10)
        self._sub_bt = self.create_subscription(
            Twist, '/cmd_vel_body_track', self._on_body_track, 10)
        # 订阅系统就绪信号 (display_node 启动完成后发布)
        self._sub_ready = self.create_subscription(
            Bool, '/system_ready', self._on_system_ready, 10)

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
        self.get_logger().info('Voice bridge ready — VOICE_MANUAL mode')

    def destroy_node(self):
        self._close_serial()
        super().destroy_node()

    # ═════════════════════════════════════════════════════════════
    # 欢迎语
    # ═════════════════════════════════════════════════════════════

    def _on_system_ready(self, msg: Bool):
        if self._welcome_played:
            return
        self._welcome_played = True
        self._write_cmd(0x02)  # A5 FA 00 82 02 00 23 FB
        self.get_logger().info('Welcome triggered — ALL SYSTEMS GO')

    # ═════════════════════════════════════════════════════════════
    # 串口
    # ═════════════════════════════════════════════════════════════

    def _write_cmd(self, cmd_id):
        cksum = (0xA5 + 0xFA + 0x00 + self._TYPE_RECV + cmd_id + 0x00) & 0xFF
        frame = bytes([0xA5, 0xFA, 0x00, self._TYPE_RECV, cmd_id, 0x00, cksum, self._TAIL])
        try:
            self._ser.write(frame)
        except serial.SerialException:
            pass

    def _close_serial(self):
        if hasattr(self, '_ser') and self._ser.is_open:
            self._ser.close()

    # ═════════════════════════════════════════════════════════════
    # body_tracking 中继
    # ═════════════════════════════════════════════════════════════

    def _on_body_track(self, msg: Twist):
        self._body_track_msg = msg
        if self._state == State.FOLLOWING and self._last_cmd_id is None:
            self._pub.publish(msg)

    # ═════════════════════════════════════════════════════════════
    # 轮询
    # ═════════════════════════════════════════════════════════════

    def _poll(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if self._last_cmd_id is not None and now - self._last_cmd_ts > self._action_duration:
            if self._state == State.FOLLOWING:
                self.get_logger().info('Voice motion done, resuming follow relay')
                if self._body_track_msg is not None:
                    self._pub.publish(self._body_track_msg)
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
                    data[i+2] == 0x00 and data[i+3] == self._TYPE_SEND and
                    data[i+7] == self._TAIL):
                    calc = (data[i] + data[i+1] + data[i+2] +
                            data[i+3] + data[i+4] + data[i+5]) & 0xFF
                    if calc == data[i+6]:
                        self._on_voice(data[i+4])
        except serial.SerialException:
            pass

    # ═════════════════════════════════════════════════════════════
    # 语音命令分发
    # ═════════════════════════════════════════════════════════════

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
    node = VoiceBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
