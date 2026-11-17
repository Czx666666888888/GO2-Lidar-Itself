"""Launch full autonomous mission: SLAM + perception + navigation + arbiter + mission."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('go2_science_perception')
    point_lio_share = get_package_share_directory('point_lio_unilidar')
    realsense_share = get_package_share_directory('realsense2_camera')

    # ── args ──────────────────────────────────────────────────────────────────
    declare_detector = DeclareLaunchArgument(
        'detector', default_value='yolo',
        description='yolo | hsv')
    declare_remote_camera = DeclareLaunchArgument(
        'remote_camera', default_value='false')
    declare_dry_run = DeclareLaunchArgument(
        'dry_run', default_value='true',
        description='true = no hardware motion output')
    declare_max_targets = DeclareLaunchArgument(
        'max_targets', default_value='6')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='false')
    declare_nav_speed = DeclareLaunchArgument(
        'nav_speed', default_value='0.4',
        description='pathFollower / localPlanner maxSpeed override')

    # ── SLAM ──────────────────────────────────────────────────────────────────
    slam = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(point_lio_share, 'launch', 'mapping_utlidar.launch')),
        launch_arguments={
            'rviz': 'false',
            'use_sim_time': 'false',
        }.items())

    # ── D435 camera (local) ───────────────────────────────────────────────────
    camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(realsense_share, 'launch', 'rs_launch.py')),
        condition=UnlessCondition(LaunchConfiguration('remote_camera')),
        launch_arguments={
            'config_file': os.path.join(pkg, 'config', 'realsense_d435.yaml'),
        }.items())

    # ── YOLO detector ─────────────────────────────────────────────────────────
    yolo = Node(
        package='go2_science_perception',
        executable='yolo_detector',
        name='yolo_detector',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('detector'), "' == 'yolo'"])),
        parameters=[os.path.join(pkg, 'config', 'yolo_detector.yaml')],
        output='screen')

    # ── HSV coarse locator (alternative) ─────────────────────────────────────
    coarse = Node(
        package='go2_science_perception',
        executable='coarse_target_locator',
        name='coarse_target_locator',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('detector'), "' == 'hsv'"])),
        parameters=[os.path.join(pkg, 'config', 'coarse_target_locator.yaml')],
        output='screen')

    target_to_map = Node(
        package='go2_science_perception',
        executable='camera_target_to_map',
        name='camera_target_to_map',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('detector'), "' == 'hsv'"])),
        parameters=[os.path.join(pkg, 'config', 'camera_target_to_map.yaml')],
        output='screen')

    # ── odometry bridge ───────────────────────────────────────────────────────
    base_odom = Node(
        package='go2_keyboard_teleop',
        executable='base_odom_node',
        name='base_odom_node',
        parameters=[{'sensor_offset_x': 0.3, 'sensor_offset_y': 0.0}],
        output='screen')

    # ── science target manager (track clustering) ─────────────────────────────
    science_manager = Node(
        package='go2_science_perception',
        executable='science_target_manager',
        name='science_target_manager',
        parameters=[os.path.join(pkg, 'config', 'science_target_manager.yaml')],
        output='screen')

    # ── approach node — remapped to /science/cmd_vel_raw ─────────────────────
    approach = Node(
        package='go2_science_perception',
        executable='science_target_approach',
        name='science_target_approach',
        parameters=[
            os.path.join(pkg, 'config', 'science_target_approach.yaml'),
            {'dry_run': LaunchConfiguration('dry_run')},
        ],
        remappings=[
            # Isolate approach output; MotionArbiter gates it onto /cmd_vel_raw
            ('/cmd_vel_raw', '/science/cmd_vel_raw'),
        ],
        output='screen')

    # ── motion arbiter — sole publisher of /cmd_vel_raw ──────────────────────
    arbiter = Node(
        package='go2_science_perception',
        executable='motion_arbiter',
        name='motion_arbiter',
        parameters=[{'dry_run': LaunchConfiguration('dry_run')}],
        output='screen')

    # ── mission manager ───────────────────────────────────────────────────────
    mission = Node(
        package='go2_science_perception',
        executable='mission_manager',
        name='mission_manager',
        parameters=[{
            'dry_run': LaunchConfiguration('dry_run'),
            'max_targets': LaunchConfiguration('max_targets'),
        }],
        output='screen')

    # ── safety gate (existing node in deploy_ws) ──────────────────────────────
    safety_gate = Node(
        package='go2_science_perception',
        executable='go2_safety_gate',    # already in deploy_ws
        name='go2_safety_gate',
        condition=UnlessCondition(LaunchConfiguration('dry_run')),
        parameters=[os.path.join(pkg, 'config', 'go2_safety_gate.yaml')],
        output='screen')

    return LaunchDescription([
        declare_detector,
        declare_remote_camera,
        declare_dry_run,
        declare_max_targets,
        declare_use_rviz,
        declare_nav_speed,
        slam,
        camera,
        yolo,
        coarse,
        target_to_map,
        base_odom,
        science_manager,
        approach,
        arbiter,
        mission,
        safety_gate,
    ])
