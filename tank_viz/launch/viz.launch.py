#!/usr/bin/env python3
"""Unified RViz digital twin: mode:=real|sim."""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tank_viz.urdf_tools import make_ghost


def _truthy(s: str) -> bool:
    return s.strip().lower() in ("true", "1", "yes", "on")


def _launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    rsp = _truthy(LaunchConfiguration("rsp").perform(context))
    ghost = _truthy(LaunchConfiguration("ghost").perform(context))
    rviz = _truthy(LaunchConfiguration("rviz").perform(context))
    static_base_arg = LaunchConfiguration("static_base").perform(context).strip().lower()

    desc_share = get_package_share_directory("tank_description")
    viz_share = get_package_share_directory("tank_viz")
    urdf_path = os.path.join(desc_share, "urdf", "tank.urdf")
    rviz_cfg = os.path.join(desc_share, "rviz", "tank.rviz")
    viz_yaml = os.path.join(viz_share, "config", "viz.yaml")

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_desc = f.read()

    actions = [
        LogInfo(msg=f"[tank_viz] mode={mode} rsp={rsp} ghost={ghost} rviz={rviz}"),
    ]

    # real: aggregate pantilt (+wheel omega) → /joint_states
    if mode == "real":
        actions.append(
            Node(
                package="tank_viz",
                executable="joint_state_aggregator",
                name="joint_state_aggregator",
                output="screen",
                parameters=[viz_yaml],
            )
        )

    # static odom→base_footprint when pantilt-only (no mega TF)
    want_static = False
    if static_base_arg in ("true", "1", "yes", "on"):
        want_static = True
    elif static_base_arg in ("auto",) and mode == "real":
        # auto: always publish static identity unless mega is known present;
        # mega will override with dynamic TF on same frames if started later —
        # prefer explicit static_base:=true for pantilt-only.
        want_static = True
    if want_static:
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_odom_base_footprint",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                    "--frame-id", "odom",
                    "--child-frame-id", "base_footprint",
                ],
                output="screen",
            )
        )

    if rsp:
        actions.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_desc}],
            )
        )

    if ghost:
        ghost_urdf = make_ghost(robot_desc, prefix="cmd_", alpha=0.3)
        actions.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="ghost_state_publisher",
                output="screen",
                parameters=[{"robot_description": ghost_urdf}],
                remappings=[
                    ("robot_description", "/cmd/robot_description"),
                    ("joint_states", "/cmd/joint_states"),
                ],
            )
        )
        ghost_params = [viz_yaml]
        if mode == "sim":
            # Sim /joint_states already in URDF frame (CAD 0).
            ghost_params.append(
                {
                    "pan_offset_rad": 0.0,
                    "tilt_offset_rad": 0.0,
                    "pan_scale": 1.0,
                    "tilt_scale": 1.0,
                    "tilt_min_rad": -0.7853981633974483,
                    "tilt_max_rad": 0.7853981633974483,
                }
            )
        actions.append(
            Node(
                package="tank_viz",
                executable="ghost_publisher",
                name="ghost_publisher",
                output="screen",
                parameters=ghost_params,
            )
        )

    actions.append(
        Node(
            package="tank_viz",
            executable="viz_markers",
            name="viz_markers",
            output="screen",
            parameters=[viz_yaml],
        )
    )

    if rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_cfg],
                output="screen",
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="real",
                description="real: aggregate joint states; sim: expect /joint_states from gz",
            ),
            DeclareLaunchArgument(
                "rsp",
                default_value="true",
                description="Start robot_state_publisher (false if sim.launch already does)",
            ),
            DeclareLaunchArgument(
                "ghost",
                default_value="true",
                description="Command-preview ghost robot + TF",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz2 with tank_description/rviz/tank.rviz",
            ),
            DeclareLaunchArgument(
                "static_base",
                default_value="auto",
                description="Publish static odom→base_footprint (true|false|auto)",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
