#!/usr/bin/env python3
"""Modular bringup: probe Mega / RealSense, start only what is present."""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tank_base.hw_probe import (
    format_probe_report,
    probe_mega,
    probe_realsense,
    resolve_mega_port,
    want_enabled,
)


def _truthy(v: str) -> bool:
    return v.strip().lower() in ("true", "1", "yes", "on")


def _setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("tank_base")
    mega_yaml = os.path.join(pkg_share, "config", "mega.yaml")
    mux_yaml = os.path.join(pkg_share, "config", "odom_mux.yaml")

    serial_port = LaunchConfiguration("serial_port").perform(context)
    mega_flag = LaunchConfiguration("mega").perform(context)
    realsense_flag = LaunchConfiguration("realsense").perform(context)
    vo_flag = LaunchConfiguration("vo").perform(context)
    odom_mode = LaunchConfiguration("odom").perform(context).strip().lower()
    publish_tf = LaunchConfiguration("publish_tf").perform(context)

    mega_r = probe_mega(serial_port)
    rs_r = probe_realsense()
    results = [mega_r, rs_r]
    report = format_probe_report(results)

    start_mega = want_enabled(mega_flag, True) if mega_flag.strip().lower() in (
        "auto",
        "",
    ) else want_enabled(mega_flag, mega_r.present)
    start_rs = want_enabled(realsense_flag, rs_r.present)
    start_vo = _truthy(vo_flag)

    resolved_port = resolve_mega_port(serial_port) or serial_port

    decisions = [
        f"  mega:      {'START' if start_mega else 'SKIP'} "
        f"(flag={mega_flag}, present={mega_r.present}, port={resolved_port})",
        f"  realsense: {'START' if start_rs else 'SKIP'} "
        f"(flag={realsense_flag}, present={rs_r.present})",
        f"  vo:        {'START' if start_vo else 'SKIP'} (flag={vo_flag})",
        f"  odom_mux:  mode={odom_mode} tf={publish_tf}",
    ]
    summary = report + "\n[tank_base] start decisions:\n" + "\n".join(decisions)

    actions = [LogInfo(msg=summary)]

    if start_mega:
        actions.append(
            Node(
                package="tank_base",
                executable="mega_bridge",
                name="mega_bridge",
                output="screen",
                parameters=[
                    mega_yaml,
                    {
                        "serial_port": resolved_port if serial_port != "auto" else "auto",
                        "publish_tf": False,
                        "odom_topic": "/odom_wheel",
                    },
                ],
            )
        )
    else:
        actions.append(
            LogInfo(msg="[tank_base] mega_bridge not started (missing or disabled)")
        )

    if start_rs:
        actions.append(
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                name="camera",
                output="screen",
                parameters=[
                    {
                        "enable_color": True,
                        "enable_depth": True,
                        "enable_infra1": False,
                        "enable_infra2": False,
                        "enable_gyro": False,
                        "enable_accel": False,
                        "enable_sync": True,
                        "rgb_camera.profile": "640x480x15",
                        "depth_module.profile": "640x480x15",
                        "align_depth.enable": True,
                        "pointcloud__neon_.enable": True,
                    }
                ],
                remappings=[
                    (
                        "/camera/camera/aligned_depth_to_color/image_raw",
                        "/camera/camera/depth/image_rect_raw",
                    ),
                ],
            )
        )
    else:
        actions.append(
            LogInfo(msg="[tank_base] RealSense not started (missing or disabled)")
        )

    if start_mega or start_rs or start_vo:
        actions.append(
            Node(
                package="tank_base",
                executable="odom_mux",
                name="odom_mux",
                output="screen",
                parameters=[
                    mux_yaml,
                    {
                        "mode": odom_mode or "auto",
                        "publish_tf": _truthy(publish_tf),
                    },
                ],
            )
        )

    if start_vo:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "vo.launch.py")
                ),
            )
        )

    if not (start_mega or start_rs or start_vo):
        actions.append(
            LogInfo(
                msg="[tank_base] WARNING: no components started — check hardware / flags"
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="auto",
                description="Mega serial port, or 'auto' for first ttyACM/ttyUSB",
            ),
            DeclareLaunchArgument(
                "mega",
                default_value="auto",
                description="auto|true|false — start mega_bridge",
            ),
            DeclareLaunchArgument(
                "realsense",
                default_value="auto",
                description="auto|true|false — start RealSense node",
            ),
            DeclareLaunchArgument(
                "vo",
                default_value="false",
                description="true|false — start rgbd_odometry → /odom_vo",
            ),
            DeclareLaunchArgument(
                "odom",
                default_value="auto",
                description="auto|wheels|vo|fuse — odom_mux output / TF source",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="odom_mux publishes TF odom → base_footprint",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
