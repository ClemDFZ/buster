#!/usr/bin/env python3
"""Bring up the unified tank control panel on :8766."""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    share = get_package_share_directory("tank_ui")
    yaml_path = os.path.join(share, "config", "ui.yaml")
    port = int(LaunchConfiguration("http_port").perform(context))
    host = LaunchConfiguration("host").perform(context)
    return [
        Node(
            package="tank_ui",
            executable="ui_server",
            name="ui_server",
            output="screen",
            parameters=[
                yaml_path,
                {"http_port": port, "host": host},
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("http_port", default_value="8766"),
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            OpaqueFunction(function=_setup),
        ]
    )
