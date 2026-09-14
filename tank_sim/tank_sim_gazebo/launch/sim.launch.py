#!/usr/bin/env python3
"""Launch Gazebo Harmonic + tank_sim spawn + bridges + optional RViz / rtabmap."""
from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _workspace_third_party_models() -> str:
    """Locate third_party aws models relative to this source tree or install."""
    # Prefer source tree under tank_ws
    here = Path(__file__).resolve()
    candidates = [
        here.parents[4] / "third_party" / "aws-robomaker-small-house-world" / "models",  # .../tank_ws/src/tank_sim/...
        here.parents[5] / "third_party" / "aws-robomaker-small-house-world" / "models",
        Path.home() / "tank_ws" / "third_party" / "aws-robomaker-small-house-world" / "models",
        Path("/home/user/tank_ws/third_party/aws-robomaker-small-house-world/models"),
    ]
    for c in candidates:
        if c.is_dir():
            return str(c)
    return str(candidates[-1])


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("tank_sim_gazebo")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world_arg = LaunchConfiguration("world").perform(context)
    gui = LaunchConfiguration("gui").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context)
    rtabmap = LaunchConfiguration("rtabmap").perform(context)
    drive = LaunchConfiguration("drive").perform(context)

    world_path = world_arg
    if not os.path.isabs(world_path):
        world_path = os.path.join(pkg_share, "worlds", world_path)

    urdf_path = os.path.join(pkg_share, "urdf", "tank_sim.urdf")
    # If drive=mecanum, regenerate is out of band; use installed URDF (velocity default).
    # Optional: read alternate urdf if present.
    if drive == "mecanum":
        alt = os.path.join(pkg_share, "urdf", "tank_sim_mecanum.urdf")
        if os.path.isfile(alt):
            urdf_path = alt

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_desc = f.read()

    models_path = _workspace_third_party_models()
    # package://tank_description/meshes/... → model://tank_description/meshes/...
    # GZ_SIM_RESOURCE_PATH must contain the parent of share/ (install/share),
    # plus AWS models/, plus the world package root for relative photos.
    pkg_share_parent = str(Path(pkg_share).parent)
    aws_root = str(Path(models_path).parent) if models_path else ""
    resource_path = ":".join(
        p
        for p in (
            models_path,
            pkg_share_parent,
            pkg_share,
            aws_root,
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        )
        if p
    )
    # Prefer distro FastCDR / RMW libs over any source-build overlays
    # (/home/user/install, ros2_jazzy) that break rtabmap ABI.
    jazzy_lib = "/opt/ros/jazzy/lib"
    ld_path = jazzy_lib
    if os.environ.get("LD_LIBRARY_PATH"):
        ld_path = jazzy_lib + ":" + os.environ["LD_LIBRARY_PATH"]


    gz_args = f"-r -s {world_path}" if gui != "true" else f"-r {world_path}"

    actions = [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("GZ_SIM_RENDER_ENGINE", "ogre2"),
        SetEnvironmentVariable("LD_LIBRARY_PATH", ld_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": gz_args,
                "on_exit_shutdown": "true",
            }.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "robot_description": robot_desc,
                }
            ],
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    arguments=[
                        "-topic",
                        "robot_description",
                        "-name",
                        "tank_sim",
                        "-z",
                        "0.1",
                        "-allow_renaming",
                        "true",
                    ],
                    output="screen",
                    parameters=[{"use_sim_time": True}],
                ),
            ],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[
                {
                    "config_file": os.path.join(pkg_share, "config", "bridge.yaml"),
                    "use_sim_time": True,
                }
            ],
            output="screen",
        ),
        # Prefer parameter_bridge for images (reliable remaps). image_bridge remaps are flaky.
        # (image_transport compressed topics come from image_transport_plugins if desired)
        Node(
            package="tank_sim_gazebo",
            executable="cmd_vel_web.py",
            name="cmd_vel_web",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        # GZ points XYZ are camera_link; gz_frame_id is optical for VO — fix stamp for RViz.
        Node(
            package="tank_sim_gazebo",
            executable="rewrite_frame_id.py",
            name="realsense_points_frame",
            parameters=[
                {
                    "frame_id": "realsense_camera_link",
                    "msg_type": "pointcloud2",
                    "use_sim_time": True,
                }
            ],
            remappings=[
                ("in", "/camera/camera/depth/color/points_gz"),
                ("out", "/camera/camera/depth/color/points"),
            ],
            output="screen",
        ),
    ]

    if rviz == "true":
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", os.path.join(pkg_share, "rviz", "sim.rviz")],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        )

    if rtabmap == "true":
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "vo_mapping.launch.py")
                )
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="small_house.sdf",
                description="World file name under share/tank_sim_gazebo/worlds or absolute path",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="false",
                description="If true, start gzclient GUI (fragile under xpra); default headless -s",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz2 with sim.rviz",
            ),
            DeclareLaunchArgument(
                "rtabmap",
                default_value="true",
                description="Start rgbd_odometry + rtabmap",
            ),
            DeclareLaunchArgument(
                "drive",
                default_value="velocity",
                description="velocity (holonomic VelocityControl) or mecanum (experimental)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
