#!/usr/bin/env python3
"""Standalone CSI → local Qwen-VL (package models + .venv)."""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("tank_vlm")
    yaml_path = os.path.join(pkg, "config", "vlm.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/csi_cam/image_raw"),
            DeclareLaunchArgument("label", default_value=""),
            DeclareLaunchArgument("auto_hz", default_value="0.0"),
            DeclareLaunchArgument("autoload", default_value="true"),
            LogInfo(msg="[tank_vlm] inline Qwen3-VL-2B (package models/)"),
            Node(
                package="tank_vlm",
                executable="vlm_bridge",
                name="vlm_bridge",
                output="screen",
                parameters=[
                    yaml_path,
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "default_label": LaunchConfiguration("label"),
                        "auto_hz": LaunchConfiguration("auto_hz"),
                        "autoload": LaunchConfiguration("autoload"),
                    },
                ],
            ),
        ]
    )
