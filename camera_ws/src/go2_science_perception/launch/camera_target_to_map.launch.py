"""Launch only the target transformer and parameterized camera extrinsic."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Create the camera-target to map transform launch description."""
    package_share = get_package_share_directory("go2_science_perception")
    config = os.path.join(
        package_share, "config", "camera_target_to_map.yaml"
    )
    return LaunchDescription([
        Node(
            package="go2_science_perception",
            executable="camera_target_to_map",
            name="camera_target_to_map",
            parameters=[config],
            output="screen",
        )
    ])
