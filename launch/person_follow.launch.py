#!/usr/bin/env python3
"""
手势唤醒人体跟随 launch

流水线:
  1. body_tracking (官方手势版: OK=唤醒, Palm=停止)
     - 内含 mipi_cam + mono2d + hand_lmk + gesture + 跟随策略
     - 发布 /cmd_vel (Twist)
  2. cmd_vel_bridge (/cmd_vel → MotorCmd → STM32)
  3. display_node (本地 HDMI 屏显)

手势操作:
  OK 手势 → 唤醒, 锁定画面中人开始跟随
  Palm 手势 → 停止跟随

用法:
  ros2 launch tracked_vehicle person_follow.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # ── 参数 ──────────────────────────────────────────
    target_dist = LaunchConfiguration('target_dist', default='2.0')

    # ── 手势人体跟随 (官方, 含相机+检测+手势+策略) ────
    body_tracking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('body_tracking'),
            '/launch/body_tracking.launch.py',
        ]),
        launch_arguments={
            'log_level': 'warn',
        }.items(),
    )

    # ── cmd_vel → MotorCmd 串口桥接 ───────────────────
    bridge_node = Node(
        package='tracked_vehicle',
        executable='cmd_vel_bridge',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{
            'serial_port': '/dev/stm32_board',
            'serial_baud': 115200,
            'linear_gain': 500.0,
            'angular_gain': 300.0,
            'cmd_timeout_s': 60.0,
        }],
    )

    # ── 本地屏显 ──────────────────────────────────────
    display_node = Node(
        package='tracked_vehicle',
        executable='display_node',
        name='display_node',
        output='screen',
        parameters=[{
            'target_dist': target_dist,
            'rotate_deg': 0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('target_dist', default_value='2.0'),
        body_tracking_launch,
        bridge_node,
        display_node,
    ])
