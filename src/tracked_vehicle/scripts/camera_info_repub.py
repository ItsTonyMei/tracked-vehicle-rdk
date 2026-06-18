#!/usr/bin/env python3
"""Republish camera_info with corrected image dimensions for GS130W GDC output.

GS130W sensor native: 1088x1280 (portrait), GDC output: 816x960.
Stereonet requires camera_info dimensions to match actual image dimensions.
This node reads the original camera_info, scales intrinsics, and republishes.

Usage:
  ros2 run tracked_vehicle camera_info_repub.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

SCALE_X = 816.0 / 1088.0
SCALE_Y = 960.0 / 1280.0


class CameraInfoRepublisher(Node):
    def __init__(self):
        super().__init__('camera_info_republisher')
        self.left_sub = self.create_subscription(
            CameraInfo, '/image_combine_raw/left/camera_info',
            self.left_callback, 10)
        self.right_sub = self.create_subscription(
            CameraInfo, '/image_combine_raw/right/camera_info',
            self.right_callback, 10)
        self.left_pub = self.create_publisher(
            CameraInfo, '/image_combine_raw/left/camera_info_corrected', 10)
        self.right_pub = self.create_publisher(
            CameraInfo, '/image_combine_raw/right/camera_info_corrected', 10)
        self.get_logger().info('CameraInfo republisher started')

    def scale_camera_info(self, msg):
        new_msg = CameraInfo()
        new_msg.header = msg.header
        new_msg.height = 960
        new_msg.width = 816
        new_msg.distortion_model = msg.distortion_model
        new_msg.d = list(msg.d)
        new_msg.k = list(msg.k)
        new_msg.k[0] *= SCALE_X
        new_msg.k[2] *= SCALE_X
        new_msg.k[4] *= SCALE_Y
        new_msg.k[5] *= SCALE_Y
        new_msg.p = list(msg.p)
        new_msg.p[0] *= SCALE_X
        new_msg.p[2] *= SCALE_X
        new_msg.p[5] *= SCALE_Y
        new_msg.p[6] *= SCALE_Y
        new_msg.r = list(msg.r)
        new_msg.binning_x = msg.binning_x
        new_msg.binning_y = msg.binning_y
        new_msg.roi = msg.roi
        return new_msg

    def left_callback(self, msg):
        corrected = self.scale_camera_info(msg)
        self.left_pub.publish(corrected)

    def right_callback(self, msg):
        corrected = self.scale_camera_info(msg)
        self.right_pub.publish(corrected)


def main():
    rclpy.init()
    node = CameraInfoRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
