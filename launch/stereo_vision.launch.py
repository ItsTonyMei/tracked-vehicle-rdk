#!/usr/bin/env python3
"""
双目视觉 launch 文件：GS130W 采集 + StereoNet 深度輐计 + Web 可视化
用法:ros2 launch tracked_vehicle stereo_vision.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mipi_width = LaunchConfiguration('mipi_width', default='640')
    mipi_height = LaunchConfiguration('mipi_height', default='352')
    mipi_fps = LaunchConfiguration('mipi_fps', default='30.0')
    mipi_rotation = LaunchConfiguration('mipi_rotation', default='90.0')
    mipi_channel_left = LaunchConfiguration('mipi_channel_left', default='0')
    mipi_channel_right = LaunchConfiguration('mipi_channel_right', default='2')
    render_type = LaunchConfiguration('render_type', default='distance')
    render_max_disp = LaunchConfiguration('render_max_disp', default='80')
    render_z_range = LaunchConfiguration('render_z_range', default='3.0')
    infer_threads = LaunchConfiguration('infer_threads', default='2')

    mipi_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('mipi_cam'),
            '/launch/mipi_cam_dual_channel.launch.py'
        ]),
        launch_arguments={
            'mipi_image_width': mipi_width,
            'mipi_image_height': mipi_height,
            'mipi_image_framerate': mipi_fps,
            'mipi_out_format': 'nv12',
            'mipi_lpwm_enable': 'True',
            'mipi_frame_ts_type': 'realtime',
            'mipi_gdc_enable': 'True',
            'mipi_channel': mipi_channel_left,
            'mipi_channel2': mipi_channel_right,
            'mipi_rotation': mipi_rotation,
            'mipi_cal_rotation': mipi_rotation,
            'log_level': 'warn',
        }.items()
    )

    stereonet_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('hobot_stereonet'),
            '/launch/stereonet_model_web_visual_v2.4_int8.launch.py'
        ]),
        launch_arguments={
            'use_mipi_cam': 'False',
            'stereo_image_topic': '/image_combine_raw',
            'camera_info_topic': '/image_combine_raw/right/camera_info',
            'left_camera_info_topic': '/image_combine_raw/left/camera_info',
            'stereonet_pub_web': 'True',
            'render_type': render_type,
            'render_max_disp': render_max_disp,
            'render_z_range': render_z_range,
            'infer_thread_num': infer_threads,
            'log_level': 'info',
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('mipi_width', default_value='640'),
        DeclareLaunchArgument('mipi_height', default_value='352'),
        DeclareLaunchArgument('mipi_fps', default_value='30.0'),
        DeclareLaunchArgument('mipi_rotation', default_value='90.0'),
        DeclareLaunchArgument('mipi_channel_left', default_value='0'),
        DeclareLaunchArgument('mipi_channel_right', default_value='2'),
        DeclareLaunchArgument('render_type', default_value='distance'),
        DeclareLaunchArgument('render_max_disp', default_value='80'),
        DeclareLaunchArgument('render_z_range', default_value='3.0'),
        DeclareLaunchArgument('infer_threads', default_value='2'),
        mipi_cam_launch,
        stereonet_launch,
    ])
