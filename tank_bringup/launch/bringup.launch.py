#!/usr/bin/env python3
"""Unified bringup: mode:=real (Humble/Orin) | sim (Jazzy/PC).

Does not exec_depend on tank_base / tank_sim_gazebo so each machine only
builds the stack it can. Missing packages fail at launch with a clear log.

Never share ROS_DOMAIN_ID between Humble and Jazzy.
"""
from __future__ import annotations

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _truthy(s: str) -> bool:
    return s.strip().lower() in ("true", "1", "yes", "on")


def _include(pkg: str, rel: str, launch_arguments=None):
    try:
        share = get_package_share_directory(pkg)
    except PackageNotFoundError:
        return [
            LogInfo(
                msg=f"[tank_bringup] missing package '{pkg}' — build it on this machine "
                f"or pick the other mode. Tried {rel}"
            )
        ]
    kwargs = {}
    if launch_arguments:
        kwargs["launch_arguments"] = launch_arguments
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, rel)),
            **kwargs,
        )
    ]


def _setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    viz = _truthy(LaunchConfiguration("viz").perform(context))
    rviz = LaunchConfiguration("rviz").perform(context)
    domain = LaunchConfiguration("ros_domain_id").perform(context).strip()
    distro = os.environ.get("ROS_DISTRO", "")
    env_domain = os.environ.get("ROS_DOMAIN_ID", "0")

    actions = []
    if domain:
        actions.append(SetEnvironmentVariable("ROS_DOMAIN_ID", domain))
        env_domain = domain

    if mode == "sim" and distro and distro != "jazzy":
        actions.append(
            LogInfo(
                msg=f"[tank_bringup] WARNING: mode:=sim but ROS_DISTRO={distro} "
                "(expected jazzy on PC)"
            )
        )
    if mode == "real" and distro and distro != "humble":
        actions.append(
            LogInfo(
                msg=f"[tank_bringup] WARNING: mode:=real but ROS_DISTRO={distro} "
                "(expected humble on Orin)"
            )
        )
    actions.append(
        LogInfo(
            msg=f"[tank_bringup] mode={mode} ROS_DISTRO={distro or '?'} "
            f"ROS_DOMAIN_ID={env_domain} — never mix Humble↔Jazzy on the same domain"
        )
    )

    if mode == "sim":
        gui = LaunchConfiguration("gui").perform(context)
        rtabmap = LaunchConfiguration("rtabmap").perform(context)
        world = LaunchConfiguration("world").perform(context)
        drive = LaunchConfiguration("drive").perform(context)
        actions.extend(
            _include(
                "tank_sim_gazebo",
                "launch/sim.launch.py",
                [
                    ("gui", gui),
                    ("rviz", "false"),
                    ("rtabmap", rtabmap),
                    ("world", world),
                    ("drive", drive),
                ],
            )
        )
        if viz:
            actions.extend(
                _include(
                    "tank_viz",
                    "launch/viz.launch.py",
                    [
                        ("mode", "sim"),
                        ("rsp", "false"),
                        ("rviz", rviz),
                        ("static_base", "false"),
                    ],
                )
            )
        return actions

    if mode != "real":
        actions.append(LogInfo(msg=f"[tank_bringup] unknown mode '{mode}' (use real|sim)"))
        return actions

    actions.extend(
        _include(
            "tank_base",
            "launch/tank_base.launch.py",
            [
                ("mega", LaunchConfiguration("mega").perform(context)),
                ("realsense", LaunchConfiguration("realsense").perform(context)),
                ("vo", LaunchConfiguration("vo").perform(context)),
                ("odom", LaunchConfiguration("odom").perform(context)),
                ("serial_port", LaunchConfiguration("serial_port").perform(context)),
                ("publish_tf", LaunchConfiguration("publish_tf").perform(context)),
            ],
        )
    )
    actions.extend(
        _include(
            "tank_pantilt",
            "launch/pantilt.launch.py",
            [
                ("bridge", LaunchConfiguration("bridge").perform(context)),
                ("csi", LaunchConfiguration("csi").perform(context)),
                ("serial_port", LaunchConfiguration("pantilt_port").perform(context)),
            ],
        )
    )
    if viz:
        actions.extend(
            _include(
                "tank_viz",
                "launch/viz.launch.py",
                [
                    ("mode", "real"),
                    ("rsp", "true"),
                    ("rviz", rviz),
                    ("static_base", LaunchConfiguration("static_base").perform(context)),
                ],
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="real",
                description="real (Humble/Orin) | sim (Jazzy/PC Gazebo)",
            ),
            DeclareLaunchArgument("viz", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value="",
                description="If set, export ROS_DOMAIN_ID for this launch. "
                "Leave empty to inherit. Never share Humble and Jazzy on one ID "
                "(recommend real=7, sim=42).",
            ),
            # real
            DeclareLaunchArgument("mega", default_value="auto"),
            DeclareLaunchArgument("realsense", default_value="auto"),
            DeclareLaunchArgument("vo", default_value="false"),
            DeclareLaunchArgument("odom", default_value="auto"),
            DeclareLaunchArgument("serial_port", default_value="auto"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("bridge", default_value="auto"),
            DeclareLaunchArgument("csi", default_value="auto"),
            DeclareLaunchArgument("pantilt_port", default_value="auto"),
            DeclareLaunchArgument("static_base", default_value="auto"),
            # sim
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rtabmap", default_value="true"),
            DeclareLaunchArgument("world", default_value="small_house.sdf"),
            DeclareLaunchArgument("drive", default_value="velocity"),
            OpaqueFunction(function=_setup),
        ]
    )
