#!/usr/bin/env python3
"""Foxglove bridge tuned for live TF: JPEG CSI, no raw images on the websocket."""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    yaml_path = os.path.join(
        get_package_share_directory("tank_viz"), "config", "foxglove.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                parameters=[yaml_path],
                arguments=["--ros-args", "--log-level", "foxglove_bridge:=warn"],
            ),
            Node(
                package="tank_viz",
                executable="jpeg_relay",
                name="jpeg_relay",
                output="screen",
                parameters=[yaml_path],
            ),
        ]
    )
