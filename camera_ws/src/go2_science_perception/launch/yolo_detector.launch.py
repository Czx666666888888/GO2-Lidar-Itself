"""Start only YOLO using an explicitly selected isolated Python interpreter."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Load configurable weights without starting HSV, SLAM or robot control."""
    share = get_package_share_directory('go2_science_perception')
    return LaunchDescription([
        DeclareLaunchArgument('python_executable', default_value='/usr/bin/python3'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            share, 'config', 'yolo_detector.yaml')),
        DeclareLaunchArgument('model_path'),
        Node(package='go2_science_perception', executable='yolo_detector',
             name='yolo_detector', prefix=[LaunchConfiguration('python_executable')],
             parameters=[LaunchConfiguration('params_file'),
                         {'model_path': LaunchConfiguration('model_path')}], output='screen'),
    ])
