"""Launch nav stack for autonomous mode: terrain + FAR + localPlanner with correct params."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    local_planner_share = get_package_share_directory('local_planner')
    terrain_share = get_package_share_directory('terrain_analysis')
    terrain_ext_share = get_package_share_directory('terrain_analysis_ext')
    far_share = get_package_share_directory('far_planner')

    declare_nav_speed = DeclareLaunchArgument(
        'nav_speed', default_value='0.4')
    declare_sensor_offset_x = DeclareLaunchArgument(
        'sensorOffsetX', default_value='0.3')

    # terrain_analysis — real-time ground/obstacle classification
    terrain = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(terrain_share, 'launch', 'terrain_analysis.launch')))

    # terrain_analysis_ext — extended occupancy for FAR
    terrain_ext = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(terrain_ext_share, 'launch', 'terrain_analysis_ext.launch')),
        launch_arguments={'checkTerrainConn': 'true'}.items())

    # FAR global planner — /goal_point -> /way_point
    far = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(far_share, 'launch', 'far_planner.launch')),
        launch_arguments={'config': 'default'}.items())

    # pathFollower source hardcodes /cmd_vel_raw; remap to /nav/cmd_vel_raw
    # so MotionArbiter remains the sole /cmd_vel_raw publisher.
    path_follower = Node(
        package='local_planner',
        executable='pathFollower',
        name='pathFollower',
        parameters=[{
            'sensorOffsetX': LaunchConfiguration('sensorOffsetX'),
            'sensorOffsetY': '0.0',
            'maxSpeed': LaunchConfiguration('nav_speed'),
            'autonomyMode': 'true',
            'autonomySpeed': LaunchConfiguration('nav_speed'),
            'goalCloseDis': '0.6',
            'is_real_robot': 'true',
            'pubSkipNum': 1,
            'twoWayDrive': False,
            'lookAheadDis': 0.5,
            'yawRateGain': 1.5,
            'stopYawRateGain': 1.5,
            'maxYawRate': 80.0,
            'maxAccel': 2.0,
            'stopDisThre': 0.2,
            'slowDwnDisThre': 0.75,
            'noRotAtGoal': True,
        }],
        remappings=[('/cmd_vel_raw', '/nav/cmd_vel_raw')],
        output='screen')

    # localPlanner generates candidate paths; pathFollower picks and executes
    local_planner = Node(
        package='local_planner',
        executable='localPlanner',
        name='localPlanner',
        parameters=[{
            'sensorOffsetX': LaunchConfiguration('sensorOffsetX'),
            'sensorOffsetY': '0.0',
            'vehicleLength': 0.7,
            'vehicleWidth': 0.4,
            'twoWayDrive': False,
            'laserVoxelSize': 0.05,
            'terrainVoxelSize': 0.2,
            'useTerrainAnalysis': True,
            'checkObstacle': True,
            'checkRotObstacle': True,
            'obstacleInflate': 0.2,
            'adjacentRange': 3.0,
            'obstacleHeightThre': 0.3,
            'groundHeightThre': 0.1,
            'minRelZ': -0.5,
            'maxRelZ': 0.25,
            'maxSpeed': LaunchConfiguration('nav_speed'),
            'dirWeight': 0.02,
            'dirThre': 90.0,
            'pathScale': 0.75,
            'minPathScale': 0.5,
            'pathScaleStep': 0.25,
            'pathCropByGoal': True,
            'autonomyMode': True,
            'autonomySpeed': LaunchConfiguration('nav_speed'),
            'goalClearRange': 0.5,
            'goalCloseDis': 0.6,
        }],
        output='screen')

    # Static TF: map <-> camera_init (Point-LIO origin frame)
    tf_map_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'])

    return LaunchDescription([
        declare_nav_speed,
        declare_sensor_offset_x,
        tf_map_camera_init,
        terrain,
        terrain_ext,
        far,
        local_planner,
        path_follower,
    ])
