from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory("go2_science_perception")
    realsense_share = get_package_share_directory("realsense2_camera")
    camera_config = os.path.join(package_share, "config", "realsense_d435.yaml")
    diagnostics_config = os.path.join(package_share, "config", "diagnostics.yaml")

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, "launch", "rs_launch.py")
        ),
        launch_arguments={"config_file": camera_config}.items(),
    )
    diagnostics = Node(
        package="go2_science_perception",
        executable="camera_diagnostics",
        name="camera_diagnostics",
        parameters=[diagnostics_config],
        output="screen",
    )
    inspector = Node(
        package="go2_science_perception",
        executable="camera_info_inspector",
        name="camera_info_inspector",
        parameters=[diagnostics_config],
        output="screen",
    )
    return LaunchDescription([camera, diagnostics, inspector])
