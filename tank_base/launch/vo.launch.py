#!/usr/bin/env python3
"""Visual odometry only (rgbd_odometry → /odom_vo, no TF).

Needs RealSense already up (stack Base) and robot_state_publisher (stack Viz)
so frame_id=base_footprint can reach the camera optical frames.

Stitches the missing URDF↔driver link: realsense_link → camera_link (identity).
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("tank_base")
    cfg = os.path.join(pkg, "config", "rgbd_odometry.yaml")

    remaps = [
        ("rgb/image", "/camera/camera/color/image_raw"),
        ("rgb/camera_info", "/camera/camera/color/camera_info"),
        ("depth/image", "/camera/camera/depth/image_rect_raw"),
        ("odom", "/odom_vo"),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_tf",
                default_value="true",
                description="static TF realsense_link → camera_link (identity)",
            ),
            LogInfo(
                msg="[vo] rgbd_odometry → /odom_vo (publish_tf=false). "
                "Start Base (camera + odom_mux) and Viz (RSP) first."
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="realsense_to_camera_link",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--yaw", "0", "--pitch", "0", "--roll", "0",
                    "--frame-id", "realsense_link",
                    "--child-frame-id", "camera_link",
                ],
                condition=IfCondition(LaunchConfiguration("camera_tf")),
            ),
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                name="rgbd_odometry",
                output="screen",
                parameters=[cfg],
                remappings=remaps,
            ),
        ]
    )
