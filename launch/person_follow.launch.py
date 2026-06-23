#!/usr/bin/env python3
"""
手势唤醒人体跟随 launch (GS130W rotation=90 适配版)

流水线:
  1. hobot_shm (零拷贝)
  2. mipi_cam (960×544, rotation=90, SC132GS)
  3. jpeg_codec (NV12→JPEG)
  4. mono2d_body_det (人体检测)
  5. hand_lmk_det (手部关键点)
  6. hand_gesture_det (OK/Palm 手势)
  7. body_tracking (跟随策略, 发布 /cmd_vel)
  8. websocket (Web可视化)
  9. cmd_vel_bridge (/cmd_vel → MotorCmd → STM32)
 10. display_node (HDMI屏显)

手势: OK→唤醒跟随, Palm→停止
用法: ros2 launch tracked_vehicle person_follow.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # ── 1. 共享内存 ───────────────────────────────────
    shm = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        get_package_share_directory('hobot_shm'), '/launch/hobot_shm.launch.py']))

    # ── 2. 相机 (GS130W, rotation=90) ─────────────────
    cam = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        get_package_share_directory('mipi_cam'), '/launch/mipi_cam.launch.py']),
        launch_arguments={
            'mipi_image_width': '960', 'mipi_image_height': '544',
            'mipi_io_method': 'shared_mem', 'mipi_frame_ts_type': 'realtime',
            'mipi_camera_calibration_file_path': 'sc132gs_calibration_90.yaml',
            'mipi_rotation': '90.0', 'mipi_cal_rotation': '90.0',
            'mipi_gdc_enable': 'True', 'mipi_out_format': 'nv12',
            'log_level': 'warn',
        }.items())

    # ── 3. JPEG 编码 (给 display + web) ───────────────
    jpeg = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        get_package_share_directory('hobot_codec'), '/launch/hobot_codec_encode.launch.py']),
        launch_arguments={
            'codec_in_mode': 'shared_mem', 'codec_out_mode': 'ros',
            'codec_sub_topic': '/hbmem_img', 'codec_pub_topic': '/image',
        }.items())

    # ── 4. 人体检测 ───────────────────────────────────
    mono2d = Node(package='mono2d_body_detection', executable='mono2d_body_detection',
        output='screen',
        parameters=[{'model_file_name': 'config/multitask_body_head_face_hand_kps_960x544.hbm',
                     'model_type': 0,
                     'ai_msg_pub_topic_name': '/hobot_mono2d_body_detection'}],
        arguments=['--ros-args', '--log-level', 'warn'])

    # ── 5. 手部关键点 ─────────────────────────────────
    hand_lmk = Node(package='hand_lmk_detection', executable='hand_lmk_detection',
        output='screen',
        parameters=[{'ai_msg_pub_topic_name': '/hobot_hand_lmk_detection',
                     'ai_msg_sub_topic_name': '/hobot_mono2d_body_detection'}],
        arguments=['--ros-args', '--log-level', 'warn'])

    # ── 6. 手势识别 ───────────────────────────────────
    hand_gesture = Node(package='hand_gesture_detection', executable='hand_gesture_detection',
        output='screen',
        parameters=[{'ai_msg_pub_topic_name': '/hobot_hand_gesture_detection',
                     'ai_msg_sub_topic_name': '/hobot_hand_lmk_detection',
                     'is_dynamic_gesture': False, 'time_interval_sec': 0.25}],
        arguments=['--ros-args', '--log-level', 'warn'])

    # ── 7. 跟随策略 (手势唤醒版) ──────────────────────
    body_track = Node(package='body_tracking', executable='body_tracking',
        output='screen',
        parameters=[{'activate_wakeup_gesture': 1,
                     'img_width': 960, 'img_height': 544,
                     'track_serial_lost_num_thr': 30,
                     'linear_velocity': 0.2, 'angular_velocity': 0.4,
                     'activate_robot_move_thr': 5}],
        arguments=['--ros-args', '--log-level', 'warn'])

    # ── 8. Web 可视化 ─────────────────────────────────
    web = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        get_package_share_directory('websocket'), '/launch/websocket.launch.py']),
        launch_arguments={
            'websocket_image_topic': '/image',
            'websocket_smart_topic': '/hobot_mono2d_body_detection',
        }.items())

    # ── 9. cmd_vel → MotorCmd ─────────────────────────
    bridge = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        get_package_share_directory('tracked_vehicle'),
        '/launch/motor_bridge.launch.py']))

    # ── 10. 屏显 ──────────────────────────────────────
    display = Node(package='tracked_vehicle', executable='display_node',
        name='display_node', output='screen',
        parameters=[{'rotate_deg': 0}])

    return LaunchDescription([
        shm, cam, jpeg, mono2d, hand_lmk, hand_gesture, body_track, web, bridge, display,
    ])
