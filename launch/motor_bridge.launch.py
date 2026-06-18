#!/usr/bin/env python3
"""
cmd_vel → MotorCmd 串口桥接 launch
将 body_tracking 或其他决策节点的 /cmd_vel 转成 MotorCmd 帧发给 STM32

用法:
  ros2 launch tracked_vehicle motor_bridge.launch.py
  ros2 launch tracked_vehicle motor_bridge.launch.py serial_port:=/dev/ttyUSB1 linear_gain:=400.0

参数:
  serial_port      — 串口设备路径 (默认 /dev/ttyUSB0, CH340N)
  serial_baud      — 波特率 (默认 115200)
  linear_gain      — 线速度→PWM 比例 (默认 500, 1m/s→±500μs)
  angular_gain     — 角速度→PWM 比例 (默认 300, 1rad/s→±300μs)
  cmd_timeout_s    — 命令超时秒数 (默认 60s, 超时发停止帧)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baud', default_value='115200'),
        DeclareLaunchArgument('linear_gain', default_value='500.0'),
        DeclareLaunchArgument('angular_gain', default_value='300.0'),
        DeclareLaunchArgument('cmd_timeout_s', default_value='60.0'),

        Node(
            package='tracked_vehicle',
            executable='cmd_vel_bridge',
            name='cmd_vel_bridge',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_baud': LaunchConfiguration('serial_baud'),
                'linear_gain': LaunchConfiguration('linear_gain'),
                'angular_gain': LaunchConfiguration('angular_gain'),
                'cmd_timeout_s': LaunchConfiguration('cmd_timeout_s'),
            }],
        ),
    ])
