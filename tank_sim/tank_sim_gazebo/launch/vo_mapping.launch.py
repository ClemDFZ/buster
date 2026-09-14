#!/usr/bin/env python3
"""VO + rtabmap SLAM for tank_sim (RealSense D415 sim topics)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("tank_sim_gazebo")
    cfg = os.path.join(pkg, "config", "rtabmap_sim.yaml")
    jazzy_lib = "/opt/ros/jazzy/lib"
    ld = jazzy_lib + (
        (":" + os.environ["LD_LIBRARY_PATH"]) if os.environ.get("LD_LIBRARY_PATH") else ""
    )

    remaps = [
        ("rgb/image", "/camera/camera/color/image_raw"),
        ("rgb/camera_info", "/camera/camera/color/camera_info"),
        ("depth/image", "/camera/camera/depth/image_rect_raw"),
    ]

    rgbd_odom = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rgbd_odometry",
        output="screen",
        parameters=[
            cfg,
            {"use_sim_time": True},
        ],
        remappings=remaps,
    )

    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[
            cfg,
            {"use_sim_time": True},
        ],
        remappings=remaps
        + [
            ("odom", "/odom"),
        ],
        arguments=["-d"],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("LD_LIBRARY_PATH", ld),
            rgbd_odom,
            rtabmap,
        ]
    )
