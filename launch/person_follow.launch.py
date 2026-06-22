#!/usr/bin/env python3
"""
distScore 人体跟随 launch — 相机(带旋转) + 检测 + 跟随 + 屏显

流水线:
  1. hobot_shm (零拷贝共享内存)
  2. mipi_cam (960×544, rotation=90, SC132GS calibration)
  3. mono2d_body_detection_without_camera (检测, 无自带相机)
  4. person_tracker (distScore → MotorCmd)
  5. display_node (本地 HDMI 屏显)

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
    # ── 跟随参数 ──────────────────────────────────────
    target_dist = LaunchConfiguration('target_dist', default='2.0')
    linear_kp = LaunchConfiguration('linear_kp', default='400.0')
    angular_kp = LaunchConfiguration('angular_kp', default='2.5')
    max_lost_frames = LaunchConfiguration('max_lost_frames', default='10')

    # ── 零拷贝共享内存 ────────────────────────────────
    shm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('hobot_shm'),
            '/launch/hobot_shm.launch.py',
        ])
    )

    # ── 相机 (GS130W SC132GS, rotation=90) ────────────
    cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('mipi_cam'),
            '/launch/mipi_cam.launch.py',
        ]),
        launch_arguments={
            'mipi_image_width': '960',
            'mipi_image_height': '544',
            'mipi_io_method': 'shared_mem',
            'mipi_frame_ts_type': 'realtime',
            'mipi_camera_calibration_file_path': 'sc132gs_calibration_90.yaml',
            'mipi_rotation': '90.0',
            'mipi_cal_rotation': '90.0',
            'mipi_gdc_enable': 'True',
            'mipi_out_format': 'nv12',
            'log_level': 'warn',
        }.items(),
    )

    # ── 人体检测 ──────────────────────────────────────
    mono2d_node = Node(
        package='mono2d_body_detection',
        executable='mono2d_body_detection',
        output='screen',
        parameters=[{
            'model_file_name': 'config/multitask_body_head_face_hand_kps_960x544.hbm',
            'model_type': 0,
            'ai_msg_pub_topic_name': '/hobot_mono2d_body_detection',
        }],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    # ── ReID 行人重识别 (跨时间保持ID) ────────────────
    reid_node = Node(
        package='reid',
        executable='reid',
        output='screen',
        parameters=[{
            'is_sync_mode': 1,
            'feed_type': 1,
            'model_file_name': 'config/reid.bin',
            'threshold': 0.70,
            'ai_msg_pub_topic_name': '/perception/detection/reid',
            'ai_msg_sub_topic_name': '/hobot_mono2d_body_detection',
        }],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # ── NV12→JPEG 编码 (给 display & web 用) ──────────
    jpeg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('hobot_codec'),
            '/launch/hobot_codec_encode.launch.py',
        ]),
        launch_arguments={
            'codec_in_mode': 'shared_mem',
            'codec_out_mode': 'ros',
            'codec_sub_topic': '/hbmem_img',
            'codec_pub_topic': '/image',
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
        DeclareLaunchArgument('linear_kp', default_value='400.0'),
        DeclareLaunchArgument('angular_kp', default_value='2.5'),
        DeclareLaunchArgument('max_lost_frames', default_value='10'),
        shm_launch,
        cam_launch,
        jpeg_launch,
        mono2d_node,
        reid_node,
        tracker_node,
        display_node,
    ])
