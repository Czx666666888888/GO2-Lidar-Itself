"""Launch D435 streams and the independent coarse blue-target locator."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Create the standalone D435 coarse-target launch description."""
    package_share = get_package_share_directory("go2_science_perception")
    realsense_share = get_package_share_directory("realsense2_camera")
    camera_config = os.path.join(package_share, "config", "realsense_d435.yaml")
    locator_config = os.path.join(
        package_share, "config", "coarse_target_locator.yaml"
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, "launch", "rs_launch.py")
        ),
        launch_arguments={"config_file": camera_config}.items(),
    )
    locator = Node(
        package="go2_science_perception",
        executable="coarse_target_locator",
        name="coarse_target_locator",
        parameters=[locator_config],
        output="screen",
    )
    return LaunchDescription([camera, locator])
