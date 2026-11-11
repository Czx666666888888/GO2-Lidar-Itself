"""Preview the MAP-anchor controller with actual RGB recording; no motion gate."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Keep dry-run selected while allowing bounded MAP/odom preview."""
    share = get_package_share_directory('go2_science_perception')
    return LaunchDescription([
        DeclareLaunchArgument('record_directory',
                              default_value=os.path.abspath('camera_ws/records')),
        Node(package='go2_science_perception', executable='approach_video_recorder',
             parameters=[{'output_directory': LaunchConfiguration('record_directory')}],
             output='screen'),
        Node(
            package='go2_science_perception', executable='science_target_approach',
            parameters=[os.path.join(share, 'config', 'science_target_approach.yaml'),
                        {'dry_run': True}], output='screen')])
