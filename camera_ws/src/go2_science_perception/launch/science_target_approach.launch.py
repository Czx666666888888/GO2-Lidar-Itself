"""Start only the approach preview controller; no gate, perception or planners."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Keep dry-run hard-selected in this first-stage launch."""
    share = get_package_share_directory('go2_science_perception')
    return LaunchDescription([Node(
        package='go2_science_perception', executable='science_target_approach',
        parameters=[os.path.join(share, 'config', 'science_target_approach.yaml'),
                    {'dry_run': True, 'allow_odom': False}], output='screen')])
