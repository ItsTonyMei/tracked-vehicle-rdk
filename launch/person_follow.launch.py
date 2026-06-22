#!/usr/bin/env python3
"""
distScore 人体跟随 launch — 检测 + 距离估算 + MotorCmd 直驱

流水线:
  1. mono2d_body_detection (官方 launch, 含 mipi_cam + 编码 + 检测)
  2. person_tracker (distScore 跟随算法 → MotorCmd → STM32)

用法:
  ros2 launch tracked_vehicle person_follow.launch.py
  ros2 launch tracked_vehicle person_follow.launch.py target_dist:=2.5 linear_kp:=300.0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # ── 跟随参数 ──────────────────────────────────────
    target_dist = LaunchConfiguration('target_dist', default='2.0')
    linear_kp = LaunchConfiguration('linear_kp', default='400.0')
    angular_kp = LaunchConfiguration('angular_kp', default='2.5')
    max_lost_frames = LaunchConfiguration('max_lost_frames', default='10')

    # ── 人体检测 (官方 launch, 含相机 + 编码) ─────────
    mono2d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('mono2d_body_detection'),
            '/launch/mono2d_body_detection.launch.py',
        ]),
        launch_arguments={
            'kps_image_width': '960',
            'kps_image_height': '544',
            'mono2d_body_pub_topic': '/hobot_mono2d_body_detection',
        }.items(),
    )

    # ── distScore 跟随节点 ────────────────────────────
    tracker_node = Node(
        package='tracked_vehicle',
        executable='person_tracker',
        name='person_tracker',
        output='screen',
        parameters=[{
            'target_dist': target_dist,
            'linear_kp': linear_kp,
            'angular_kp': angular_kp,
            'max_lost_frames': max_lost_frames,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('target_dist', default_value='2.0'),
        DeclareLaunchArgument('linear_kp', default_value='400.0'),
        DeclareLaunchArgument('angular_kp', default_value='2.5'),
        DeclareLaunchArgument('max_lost_frames', default_value='10'),
        mono2d_launch,
        tracker_node,
    ])
