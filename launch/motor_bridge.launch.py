#!/usr/bin/env python3
"""
cmd_vel → MotorCmd 串口桥接 launch
将 body_tracking 或其他决策节点的 /cmd_vel 转成 MotorCmd 帧发给 STM32

用法:
  ros2 launch tracked_vehicle motor_bridge.launch.py
  ros2 launch tracked_vehicle motor_bridge.launch.py serial_port:=/dev/ttyUSB1 linear_gain:=400.0

参数:
  serial_port      — 串口设备路径 (默认 /dev/stm32_board, CH340N)
  serial_baud      — 波特率 (默认 115200)
  linear_gain      — 线速度→PWM 比例 (默认 500, 1m/s→±500μs)
  angular_gain     — 角速度→PWM 比例 (默认 300, 1rad/s→±300μs)
  steering_invert  — 转向方向取反 (默认 true, angular.z>0→左转)
  cmd_timeout_s    — 命令超时秒数 (默认 60s, 超时发停止帧)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/stm32_board'),
        DeclareLaunchArgument('serial_baud', default_value='115200'),
        DeclareLaunchArgument('linear_gain', default_value='500.0'),
        DeclareLaunchArgument('angular_gain', default_value='300.0'),
        DeclareLaunchArgument('steering_invert', default_value='true'),
        DeclareLaunchArgument('cmd_timeout_s', default_value='60.0'),

        Node(
            package='tracked_vehicle',
            executable='motor_bridge',
            name='motor_bridge',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_baud': LaunchConfiguration('serial_baud'),
                'linear_gain': LaunchConfiguration('linear_gain'),
                'angular_gain': LaunchConfiguration('angular_gain'),
                'steering_invert': LaunchConfiguration('steering_invert'),
                'cmd_timeout_s': LaunchConfiguration('cmd_timeout_s'),
            }],
        ),
    ])
