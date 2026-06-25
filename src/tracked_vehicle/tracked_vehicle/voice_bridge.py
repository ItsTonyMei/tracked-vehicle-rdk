#!/usr/bin/env python3
"""
AI 语音模块 → /cmd_vel 桥接节点
通过 UART 接收 CI1302 语音识别结果, 映射为车辆控制指令并发布到 /cmd_vel

协议: AA 55 [STATUS] [CMD_ID] FB @ 115200 bps
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time


class VoiceBridge(Node):
    """语音识别 → /cmd_vel 桥接.

    订阅语音模块串口输出, 识别到命令词时发布对应 Twist 指令。
    动作持续 3 秒后自动停止 (可配置).
    """

    # 出厂固件命令词映射 (ID → (动作名, (vx, vy, angular_z)))
    CMD_MAP = {
        0: ('STOP', (0.0, 0.0, 0.0)),
        2: ('STOP', (0.0, 0.0, 0.0)),
        4: ('FORWARD', (0.5, 0.0, 0.0)),
        5: ('BACKWARD', (-0.3, 0.0, 0.0)),
        6: ('TURN_LEFT', (0.2, 0.0, 0.4)),
        7: ('TURN_RIGHT', (0.2, 0.0, -0.4)),
        8: ('SPIN_LEFT', (0.0, 0.0, 0.5)),
        9: ('SPIN_RIGHT', (0.0, 0.0, -0.5)),
    }

    def __init__(self):
        super().__init__('voice_bridge')

        port = self.declare_parameter('voice_port', '/dev/ttyUSB1').value
        baud = self.declare_parameter('voice_baud', 115200).value
        self._action_duration = self.declare_parameter('action_duration_s', 3.0).value

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._last_cmd_ts = 0.0
        self._last_cmd_id = None

        try:
            self._ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Voice module opened: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open voice port {port}: {e}')
            raise

        # 初始化语音模块 (发送 init 命令)
        time.sleep(0.5)
        self._write_cmd(0x67)
        self.get_logger().info('Voice bridge ready. Speak a command.')

        # 轮询语音数据 @ 20Hz
        self._timer = self.create_timer(0.05, self._poll)

    def __del__(self):
        if hasattr(self, '_ser') and self._ser.is_open:
            self._ser.close()

    def _write_cmd(self, cmd_id):
        """发送 5 字节帧触发语音模块播报."""
        frame = bytes([0xAA, 0x55, 0xFF, cmd_id, 0xFB])
        try:
            self._ser.write(frame)
            time.sleep(0.005)
            self._ser.flushInput()
        except serial.SerialException:
            pass

    def _poll(self):
        """轮询语音模块串口, 解析识别结果."""
        now = self.get_clock().now().nanoseconds / 1e9

        # 动作超时 → 自动停止
        if self._last_cmd_id is not None and now - self._last_cmd_ts > self._action_duration:
            self._publish_cmd('AUTO_STOP', (0.0, 0.0, 0.0))
            self._last_cmd_id = None

        try:
            count = self._ser.inWaiting()
            if not count:
                return
            data = self._ser.read(count)
            hex_str = data.hex()
            if hex_str.startswith('aa55') and len(hex_str) >= 8:
                cmd_id = int(hex_str[6:8], 16)
                self._ser.flushInput()
                self._on_voice(cmd_id)
        except serial.SerialException:
            pass

    def _on_voice(self, cmd_id):
        """语音命令处理."""
        if cmd_id not in self.CMD_MAP:
            self.get_logger().debug(f'Unknown command ID: {cmd_id}')
            return

        name, (vx, vy, az) = self.CMD_MAP[cmd_id]
        now = self.get_clock().now().nanoseconds / 1e9
        self._publish_cmd(name, (vx, vy, az))
        self._last_cmd_ts = now
        self._last_cmd_id = cmd_id
        self.get_logger().info(f'VOICE: {name} (ID={cmd_id})')

    def _publish_cmd(self, name, vel):
        """发布 /cmd_vel 并触发语音确认播报."""
        msg = Twist()
        msg.linear.x = float(vel[0])
        msg.linear.y = float(vel[1])
        msg.angular.z = float(vel[2])
        self._pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(VoiceBridge())
    rclpy.shutdown()
