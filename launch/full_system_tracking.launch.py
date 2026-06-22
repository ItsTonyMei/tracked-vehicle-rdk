#!/usr/bin/env python3
"""
全系统人体跟随启动 (GS130W 适配版)

启动流程:
  1. mono2d_body_detection (960x544, GS130W SC132GS @ rotation=90)
  2. body_tracking (人体跟随策略, 发布 /cmd_vel)
  3. cmd_vel_bridge (cmd_vel → MotorCmd → STM32)

用法:
  ros2 launch tracked_vehicle full_system_tracking.launch.py
  ros2 launch tracked_vehicle full_system_tracking.launch.py serial_port:=/dev/ttyUSB1
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # ── 参数 ──────────────────────────────────────────────
    serial_port = LaunchConfiguration('serial_port', default='/dev/stm32_board')
    linear_gain = LaunchConfiguration('linear_gain', default='500.0')
    angular_gain = LaunchConfiguration('angular_gain', default='300.0')
    cmd_timeout = LaunchConfiguration('cmd_timeout_s', default='60.0')
    cam_type = os.environ.get('CAM_TYPE', 'mipi')

    # ── 人体检测 + 跟踪 (官方 body_tracking) ──────────────
    # GS130W 适配参数: sc132gs calibration, rotation=90
    body_tracking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('body_tracking'),
            '/launch/body_tracking_without_gesture.launch.py',
        ]),
        launch_arguments={
            'mipi_camera_calibration_file_path': 'sc132gs_calibration_90.yaml',
            'mipi_rotation': '90.0',
            'mipi_cal_rotation': '90.0',
        }.items(),
    )

    # ── 串口桥接 ──────────────────────────────────────────
    bridge_node = Node(
        package='tracked_vehicle',
        executable='cmd_vel_bridge',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'serial_baud': 115200,
            'linear_gain': linear_gain,
            'angular_gain': angular_gain,
            'cmd_timeout_s': cmd_timeout,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/stm32_board'),
        DeclareLaunchArgument('linear_gain', default_value='500.0'),
        DeclareLaunchArgument('angular_gain', default_value='300.0'),
        DeclareLaunchArgument('cmd_timeout_s', default_value='60.0'),
        body_tracking_launch,
        bridge_node,
    ])
