#!/usr/bin/env python3
"""Launch Gazebo Harmonic + tank_sim spawn + bridges + nav/pantilt contract nodes."""
from __future__ import annotations

import os
import re
import subprocess
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _workspace_third_party_models() -> str:
    """Locate third_party aws models relative to this source tree or install."""
    here = Path(__file__).resolve()
    candidates = [
        here.parents[4] / "third_party" / "aws-robomaker-small-house-world" / "models",
        here.parents[5] / "third_party" / "aws-robomaker-small-house-world" / "models",
        Path.home() / "tank_ws" / "third_party" / "aws-robomaker-small-house-world" / "models",
    ]
    for c in candidates:
        if c.is_dir():
            return str(c)
    return str(candidates[0])


def _cleanup_script() -> Path | None:
    cands = [
        Path(__file__).resolve().parents[1] / "scripts" / "sim_cleanup.sh",
        Path(get_package_prefix("tank_sim_gazebo")) / "lib" / "tank_sim_gazebo" / "sim_cleanup.sh",
    ]
    for c in cands:
        if c.is_file():
            return c
    return None


def _world_sdf_name(world_path: str) -> str:
    try:
        text = Path(world_path).read_text(encoding="utf-8")
    except OSError:
        return Path(world_path).stem
    m = re.search(r"<world\b[^>]*\bname\s*=\s*['\"]([^'\"]+)", text)
    return m.group(1) if m else Path(world_path).stem


def _materialize_bridge(pkg_share: str, world_name: str, model_name: str) -> str:
    src = Path(pkg_share) / "config" / "bridge.yaml"
    text = src.read_text(encoding="utf-8")
    text = text.replace("__WORLD__", world_name).replace("__MODEL__", model_name)
    fd, path = tempfile.mkstemp(prefix="tank_sim_bridge_", suffix=".yaml")
    os.close(fd)
    Path(path).write_text(text, encoding="utf-8")
    return path


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("tank_sim_gazebo")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world_arg = LaunchConfiguration("world").perform(context)
    gui = LaunchConfiguration("gui").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context)
    rtabmap = LaunchConfiguration("rtabmap").perform(context)
    drive = LaunchConfiguration("drive").perform(context)
    model_name = LaunchConfiguration("model_name").perform(context).strip() or "tank_sim"

    world_path = world_arg
    if not os.path.isabs(world_path):
        world_path = os.path.join(pkg_share, "worlds", world_path)

    urdf_path = os.path.join(pkg_share, "urdf", "tank_sim.urdf")
    if drive == "mecanum":
        alt = os.path.join(pkg_share, "urdf", "tank_sim_mecanum.urdf")
        if os.path.isfile(alt):
            urdf_path = alt

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_desc = f.read()

    models_path = _workspace_third_party_models()
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
    jazzy_lib = "/opt/ros/jazzy/lib"
    ld_path = jazzy_lib
    if os.environ.get("LD_LIBRARY_PATH"):
        ld_path = jazzy_lib + ":" + os.environ["LD_LIBRARY_PATH"]

    cleanup = _cleanup_script()
    if cleanup is not None:
        subprocess.run(["bash", str(cleanup)], check=False)
    else:
        # Fallback: free teleop port at least.
        subprocess.run(["fuser", "-k", "8768/tcp"], check=False, capture_output=True)

    world_name = _world_sdf_name(world_path)
    bridge_yaml = _materialize_bridge(pkg_share, world_name, model_name)
    partition = os.environ.get("GZ_PARTITION") or f"tank_sim_{os.getpid()}"
    distro = os.environ.get("ROS_DISTRO", "?")
    domain = os.environ.get("ROS_DOMAIN_ID", "0")

    gz_args = f"-r -s {world_path}" if gui != "true" else f"-r {world_path}"
    use_rtabmap = rtabmap == "true"

    actions = [
        LogInfo(
            msg=f"[tank_sim] distro={distro} ROS_DOMAIN_ID={domain} "
            f"GZ_PARTITION={partition} world={world_name} model={model_name} "
            f"— do not share this domain with Humble/Orin"
        ),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("GZ_SIM_RENDER_ENGINE", "ogre2"),
        SetEnvironmentVariable("GZ_PARTITION", partition),
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
            period=3.5,
            actions=[
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    arguments=[
                        "-topic",
                        "robot_description",
                        "-name",
                        model_name,
                        "-z",
                        "0.1",
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
                    "config_file": bridge_yaml,
                    "use_sim_time": True,
                }
            ],
            output="screen",
        ),
        Node(
            package="tank_sim_gazebo",
            executable="cmd_vel_web.py",
            name="cmd_vel_web",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="tank_sim_gazebo",
            executable="pantilt_sim_adapter.py",
            name="pantilt_sim_adapter",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="tank_sim_gazebo",
            executable="sim_nav_contract.py",
            name="sim_nav_contract",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    # rtabmap VO owns /odom + TF; otherwise GT fills the nav contract.
                    "publish_nav_odom": not use_rtabmap,
                    "publish_tf": not use_rtabmap,
                    "publish_imu": True,
                }
            ],
        ),
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

    if use_rtabmap:
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
                description="Start rgbd_odometry + rtabmap (owns /odom + TF). "
                "If false, sim_nav_contract publishes /odom from /odom_gt.",
            ),
            DeclareLaunchArgument(
                "drive",
                default_value="velocity",
                description="velocity (holonomic VelocityControl) or mecanum (experimental)",
            ),
            DeclareLaunchArgument(
                "model_name",
                default_value="tank_sim",
                description="GZ model name (must match bridge /model/__MODEL__ topics)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
