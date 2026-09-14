from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("tank_track")
    params = os.path.join(share, "config", "track.yaml")
    return LaunchDescription(
        [
            Node(
                package="tank_track",
                executable="track_server",
                name="track_server",
                output="screen",
                parameters=[params],
            )
        ]
    )
