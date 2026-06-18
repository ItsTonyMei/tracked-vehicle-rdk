#!/usr/bin/env python3
"""
cmd_vel → MotorCmd 串口桥接节点
订阅 ROS2 /cmd_vel (Twist), 转换为 6 字节 MotorCmd 帧, 通过 UART 下发给 STM32

MotorCmd 帧格式:
  [0xAA][th_lo][th_hi][st_lo][st_hi][CRC8]  6 bytes @ 115200 bps
  throttle/steering: uint16 LE, 1500μs=停止, 1000-2000μs 范围
  CRC8: poly=0x07, init=0x00, 覆盖 byte1-4
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct
import os


def crc8(data: bytes) -> int:
    """CRC-8-MAXIM (poly=0x07, init=0x00)"""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc << 1) ^ 0x07 if crc & 0x80 else crc << 1
    return crc & 0xFF


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # 串口配置
        port = self.declare_parameter('serial_port', '/dev/ttyUSB0').value
        baud = self.declare_parameter('serial_baud', 115200).value

        # 速度 → PWM 映射参数
        self.linear_gain = self.declare_parameter('linear_gain', 500.0).value
        self.angular_gain = self.declare_parameter('angular_gain', 300.0).value
        self.pwm_center = 1500
        self.pwm_min = 1000
        self.pwm_max = 2000

        # 打开串口
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'串口已打开: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().fatal(f'无法打开串口 {port}: {e}')
            raise

        # 订阅 /cmd_vel
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)

        # 命令超时定时器 (60s 无新命令 → 发停止帧)
        self.timeout = self.declare_parameter('cmd_timeout_s', 60.0).value
        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(0.5, self.watchdog)

    def cmd_cb(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        throttle = self.pwm_center + int(msg.linear.x * self.linear_gain)
        steering = self.pwm_center + int(msg.angular.z * self.angular_gain)

        throttle = max(self.pwm_min, min(self.pwm_max, throttle))
        steering = max(self.pwm_min, min(self.pwm_max, steering))

        self._send(throttle, steering)

    def watchdog(self):
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if dt > self.timeout:
            self._send(self.pwm_center, self.pwm_center)

    def _send(self, throttle: int, steering: int):
        payload = struct.pack('<HH', throttle, steering)  # 4 bytes LE
        frame = b'\xAA' + payload + bytes([crc8(payload)])
        try:
            self.ser.write(frame)
        except serial.SerialException as e:
            self.get_logger().error(f'串口写入失败: {e}')


def main():
    rclpy.init()
    rclpy.spin(CmdVelBridge())
    rclpy.shutdown()
