#!/usr/bin/env python3
"""
motor_bridge — Twist → MotorCmd 串口桥接节点 (纯执行, 零决策)

订阅 ROS2 /cmd_vel (Twist), 转换为 6 字节 MotorCmd 帧, 通过 UART 下发给 STM32

MotorCmd 帧格式:
  [0xAA][th_lo][th_hi][st_lo][st_hi][CRC8]  6 bytes @ 115200 bps
  throttle/steering: uint16 LE, 1500us=停止, 1000-2000us 范围
  CRC8: poly=0x07, init=0x00, 覆盖 byte1-4

附带诊断: STM32 在同一 USART1 上输出调试打印 (状态行/安全事件/启动 banner),
本节点读取并将关键事件 ([SAFE]/[SBUS]/[MODE]/ARM/启动 banner 等) 转发到 ROS 日志 —
STM32 意外复位 (IWDG/掉电) 会在 ROS 日志中留下 banner 痕迹.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct


def crc8(data: bytes) -> int:
    """CRC-8 (poly=0x07, init=0x00, 与 STM32 固件一致)"""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07 if crc & 0x80 else crc << 1) & 0xFF
    return crc


class MotorBridge(Node):
    """执行层: /cmd_vel → Serial MotorCmd. 不做任何决策."""

    def __init__(self):
        super().__init__('motor_bridge')

        port = self.declare_parameter('serial_port', '/dev/stm32_board').value
        baud = self.declare_parameter('serial_baud', 115200).value

        self.linear_gain = self.declare_parameter('linear_gain', 500.0).value
        self.angular_gain = self.declare_parameter('angular_gain', 300.0).value
        self.steering_invert = self.declare_parameter('steering_invert', True).value
        self.pwm_center = 1500
        self.pwm_min = 1000
        self.pwm_max = 2000

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'串口已打开: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().fatal(f'无法打开串口 {port}: {e}')
            raise

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)

        self.timeout = self.declare_parameter('cmd_timeout_s', 60.0).value
        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(min(5.0, self.timeout / 10.0), self.watchdog)

        # STM32 调试输出转发 (USART1 与 MotorCmd 共享, STM32 print → 本端口 RX)
        self._stm32_rx_buf = b''
        self.create_timer(0.5, self._read_stm32_log)

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()

    def cmd_cb(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        throttle = self.pwm_center + int(msg.linear.x * self.linear_gain)
        sign = -1 if self.steering_invert else 1
        steering = self.pwm_center + sign * int(msg.angular.z * self.angular_gain)

        throttle = max(self.pwm_min, min(self.pwm_max, throttle))
        steering = max(self.pwm_min, min(self.pwm_max, steering))

        self._send(throttle, steering)

    def watchdog(self):
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if dt > self.timeout:
            self._send(self.pwm_center, self.pwm_center)

    def _send(self, throttle: int, steering: int):
        payload = struct.pack('<HH', throttle, steering)
        frame = b'\xAA' + payload + bytes([crc8(payload)])
        try:
            self.ser.write(frame)
        except (serial.SerialException, OSError):
            if self._try_reconnect():
                try:
                    self.ser.write(frame)
                except (serial.SerialException, OSError):
                    self.get_logger().warn('STM32: MotorCmd write failed after reconnect')
            # else: reconnect failed → logged in _try_reconnect

    def _try_reconnect(self):
        try:
            if self.ser.is_open:
                self.ser.close()
            self.ser.open()
            self.ser.reset_input_buffer()
            self.get_logger().info('串口已重新连接')
            return True
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f'串口重连失败: {e}')
            return False

    # ── STM32 调试输出转发 ──
    # 周期状态行 (含 'thr=') 不转发; 关键事件 ([SAFE]/[SBUS]/[MODE]/ARM/
    # 启动 banner 等) 以 WARN 转发 — STM32 复位时 banner 会出现在 ROS 日志中
    _STM32_LOG_KEYS = ('[SAFE]', '[SBUS]', '[MODE]', '[ESC]', '[IMU]', '[MCU]',
                       '[X5]', 'ARMED', 'DISARMED', 'READY.', 'STM32 V3.0')

    def _read_stm32_log(self):
        try:
            n = self.ser.in_waiting
            if n:
                self._stm32_rx_buf += self.ser.read(n)
        except (serial.SerialException, OSError):
            return
        while b'\n' in self._stm32_rx_buf:
            line, self._stm32_rx_buf = self._stm32_rx_buf.split(b'\n', 1)
            text = line.decode('utf-8', errors='replace').strip()
            if not text or 'thr=' in text:
                continue
            if any(k in text for k in self._STM32_LOG_KEYS):
                self.get_logger().warn(f'STM32: {text}')
        if len(self._stm32_rx_buf) > 512:  # 无换行时防 buffer 膨胀
            self._stm32_rx_buf = self._stm32_rx_buf[-256:]


def main():
    rclpy.init()
    node = MotorBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
