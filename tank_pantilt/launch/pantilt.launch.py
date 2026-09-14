#!/usr/bin/env python3
"""Standalone pantilt + CSI bringup (no mobile base / Mega / RealSense)."""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tank_pantilt.hw_probe import (
    format_probe_report,
    probe_argus,
    probe_esp,
    probe_mjpeg,
    resolve_esp_port,
    want_enabled,
)


def _setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("tank_pantilt")
    yaml_path = os.path.join(pkg_share, "config", "pantilt.yaml")

    serial_port = LaunchConfiguration("serial_port").perform(context)
    bridge_flag = LaunchConfiguration("bridge").perform(context)
    csi_flag = LaunchConfiguration("csi").perform(context)
    csi_owner = LaunchConfiguration("csi_owner").perform(context)
    mjpeg_url = LaunchConfiguration("mjpeg_url").perform(context)

    esp_r = probe_esp(serial_port)
    mjpeg_r = probe_mjpeg(mjpeg_url)
    argus_r = probe_argus()

    effective_owner = csi_owner
    owner_note = ""
    if csi_owner == "auto":
        if mjpeg_r.present:
            effective_owner = "pantilt"
            owner_note = " (auto→pantilt MJPEG)"
        elif argus_r.present:
            effective_owner = "ros"
            owner_note = " (auto→ros Argus)"
        else:
            effective_owner = "pantilt"
            owner_note = " (auto→pantilt, neither present)"
    elif csi_owner == "pantilt" and not mjpeg_r.present and argus_r.present:
        effective_owner = "ros"
        owner_note = " (fallback pantilt→ros Argus)"

    csi_present = mjpeg_r.present if effective_owner == "pantilt" else argus_r.present
    report = format_probe_report([esp_r, mjpeg_r, argus_r])

    start_bridge = want_enabled(bridge_flag, esp_r.present)
    start_csi = want_enabled(csi_flag, csi_present)
    resolved = resolve_esp_port(serial_port) or serial_port

    decisions = [
        f"  bridge: {'START' if start_bridge else 'SKIP'} "
        f"(flag={bridge_flag}, esp={esp_r.present}, port={resolved})",
        f"  csi:    {'START' if start_csi else 'SKIP'} "
        f"(flag={csi_flag}, owner={effective_owner}{owner_note}, present={csi_present})",
    ]
    summary = report + "\n[tank_pantilt] start decisions:\n" + "\n".join(decisions)
    actions = [LogInfo(msg=summary)]

    if start_bridge:
        actions.append(
            Node(
                package="tank_pantilt",
                executable="pantilt_bridge",
                name="pantilt_bridge",
                output="screen",
                parameters=[
                    yaml_path,
                    {"serial_port": "auto" if serial_port == "auto" else resolved},
                ],
            )
        )
    else:
        actions.append(LogInfo(msg="[tank_pantilt] pantilt_bridge not started"))

    if start_csi:
        if effective_owner == "ros":
            actions.append(
                Node(
                    package="tank_pantilt",
                    executable="csi_argus",
                    name="csi_argus",
                    output="screen",
                    parameters=[
                        {
                            "width": 1280,
                            "height": 720,
                            "fps": 30,
                            "frame_id": "csi_optical_frame",
                            "hfov_deg": 62.2,
                        }
                    ],
                )
            )
        else:
            actions.append(
                Node(
                    package="tank_pantilt",
                    executable="csi_republish",
                    name="csi_republish",
                    output="screen",
                    parameters=[
                        {
                            "mjpeg_url": mjpeg_url,
                            "width": 1280,
                            "height": 720,
                            "frame_id": "csi_optical_frame",
                            "hfov_deg": 62.2,
                            "rate_hz": 30.0,
                        }
                    ],
                )
            )
    else:
        actions.append(LogInfo(msg="[tank_pantilt] CSI not started"))

    if not (start_bridge or start_csi):
        actions.append(LogInfo(msg="[tank_pantilt] WARNING: no components started"))

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="auto",
                description="ESP serial port (prefer ttyUSB) or auto",
            ),
            DeclareLaunchArgument(
                "bridge",
                default_value="auto",
                description="auto|true|false — pantilt_bridge (ESP)",
            ),
            DeclareLaunchArgument(
                "csi",
                default_value="auto",
                description="auto|true|false — CSI image publisher",
            ),
            DeclareLaunchArgument(
                "csi_owner",
                default_value="auto",
                description="auto|pantilt|ros — MJPEG from pantilt_slave vs Argus",
            ),
            DeclareLaunchArgument(
                "mjpeg_url",
                default_value="http://127.0.0.1:5003/video_feed/color",
                description="MJPEG when csi_owner:=pantilt",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
