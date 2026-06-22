#!/usr/bin/env python3
"""
distScore 人体跟随节点 — RDK X5 版

移植自 ESP32 FollowLogic，核心改进:
  - 使用 mono2d_body_detection 的 bbox 替代 OpenMV VIS 帧
  - 误差 = 目标距离 - 当前距离 → 比例控制油门
  - bbox 水平偏移 → 比例控制转向
  - 直接输出 MotorCmd 帧 (绕过 /cmd_vel)

距离估算: 基于 bbox 宽度反比 (人体肩宽 ~0.5m 恒定)
  dist_m = (FOCAL_PX * REAL_WIDTH_M) / bbox_width_px

控制逻辑:
  dist_error >  deadzone → 前进 (throttle > 1500)
  dist_error < -deadzone → 后退/停止
  x_offset  >  deadzone → 转向跟踪
"""

import rclpy
from rclpy.node import Node
from ai_msgs.msg import PerceptionTargets
import serial
import struct
import math


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc << 1) ^ 0x07 if crc & 0x80 else crc << 1
    return crc & 0xFF


class PersonTracker(Node):
    def __init__(self):
        super().__init__('person_tracker')

        # ── 串口 ──────────────────────────────────────
        port = self.declare_parameter('serial_port', '/dev/stm32_board').value
        baud = self.declare_parameter('serial_baud', 115200).value
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'串口: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().fatal(f'串口失败 {port}: {e}')
            raise

        # ── 跟随参数 ──────────────────────────────────
        # 距离估算: bbox_width_1m / bbox_width = dist
        self.bbox_ref_width = self.declare_parameter('bbox_ref_width', 500.0).value
        self.bbox_ref_dist = self.declare_parameter('bbox_ref_dist', 2.0).value

        # 目标距离 (米), 死区 (米)
        self.target_dist = self.declare_parameter('target_dist', 2.0).value
        self.dist_deadzone = self.declare_parameter('dist_deadzone', 0.3).value
        self.angle_deadzone = self.declare_parameter('angle_deadzone', 40).value

        # PID 增益
        self.linear_kp = self.declare_parameter('linear_kp', 400.0).value
        self.angular_kp = self.declare_parameter('angular_kp', 2.5).value

        # PWM 限幅
        self.pwm_center = 1500
        self.pwm_min = 1000
        self.pwm_max = 2000
        self.max_linear_offset = self.declare_parameter('max_linear_offset', 400).value
        self.max_angular_offset = self.declare_parameter('max_angular_offset', 300).value

        # 图像尺寸 (body_tracking 默认 960×544)
        self.img_width = self.declare_parameter('img_width', 960).value
        self.img_height = self.declare_parameter('img_height', 544).value

        # ── 状态 ──────────────────────────────────────
        self.target_lost_frames = 0
        self.max_lost_frames = self.declare_parameter('max_lost_frames', 10).value
        self.last_throttle = self.pwm_center
        self.last_steering = self.pwm_center

        # ── 安全超时 ──────────────────────────────────
        timeout_s = self.declare_parameter('cmd_timeout_s', 60.0).value
        self.timer = self.create_timer(1.0, self.watchdog)
        self.last_cmd_time = self.get_clock().now()
        self.timeout_s = timeout_s

        # ── 订阅人检测 ────────────────────────────────
        self.sub = self.create_subscription(
            PerceptionTargets,
            '/perception/detection/reid',
            self.detection_cb,
            10)

        self.get_logger().info('person_tracker 就绪')

    def detection_cb(self, msg: PerceptionTargets):
        now = self.get_clock().now()

        # 找跟踪中的人 (track_id 有效且 type=person)
        person = None
        for t in msg.targets:
            if t.type == 'person' and t.track_id > 0:
                person = t
                break

        if person is None:
            self.target_lost_frames += 1
            if self.target_lost_frames > self.max_lost_frames:
                self._send(self.pwm_center, self.pwm_center)
            return

        self.target_lost_frames = 0
        self.last_cmd_time = now

        roi = person.rois[0].rect
        bbox_w = roi.width
        bbox_h = roi.height
        bbox_cx = roi.x_offset + bbox_w / 2.0
        bbox_cy = roi.y_offset + bbox_h / 2.0

        # ── 距离估算 (bbox 宽度反比) ─────────────────
        if bbox_w > 0:
            dist = (self.bbox_ref_width * self.bbox_ref_dist) / bbox_w
        else:
            dist = self.target_dist

        # ── distScore 误差计算 ────────────────────────
        dist_error = dist - self.target_dist
        center_error = bbox_cx - (self.img_width / 2.0)

        # ── 比例控制 ──────────────────────────────────
        if abs(dist_error) < self.dist_deadzone:
            throttle = self.pwm_center
        else:
            linear_offset = dist_error * self.linear_kp
            linear_offset = max(-self.max_linear_offset, min(self.max_linear_offset, linear_offset))
            throttle = self.pwm_center + int(linear_offset)

        if abs(center_error) < self.angle_deadzone:
            # 人基本居中, 直线
            steering = self.pwm_center
        else:
            # 人在画面右侧 → 应右转 → steering > 1500
            angular_offset = center_error * self.angular_kp
            angular_offset = max(-self.max_angular_offset, min(self.max_angular_offset, angular_offset))
            steering = self.pwm_center + int(angular_offset)

        throttle = max(self.pwm_min, min(self.pwm_max, throttle))
        steering = max(self.pwm_min, min(self.pwm_max, steering))

        self.last_throttle = throttle
        self.last_steering = steering

        self._send(throttle, steering)

        self.get_logger().info(
            f'dist={dist:.1f}m err={dist_error:+.1f} '
            f'bbox={bbox_w}x{bbox_h}@{int(bbox_cx)},{int(bbox_cy)} '
            f'track={person.track_id} '
            f'PWM thr={throttle} str={steering}')

    def watchdog(self):
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if dt > self.timeout_s:
            self._send(self.pwm_center, self.pwm_center)

    def _send(self, throttle: int, steering: int):
        payload = struct.pack('<HH', throttle, steering)
        frame = b'\xAA' + payload + bytes([crc8(payload)])
        try:
            self.ser.write(frame)
        except serial.SerialException as e:
            self.get_logger().error(f'串口写入失败: {e}')


def main():
    rclpy.init()
    rclpy.spin(PersonTracker())
    rclpy.shutdown()
