"""Launch radar SLAM, D435 target mapping and RViz without navigation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def static_tf(name, parent, child, xyz):
    """Create a zero-rotation static transform publisher action."""
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--x", str(xyz[0]), "--y", str(xyz[1]), "--z", str(xyz[2]),
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", parent, "--child-frame-id", child,
        ],
        output="screen",
    )


def generate_launch_description():
    """Create the read-only SLAM, camera, target and RViz composition."""
    package_share = get_package_share_directory("go2_science_perception")
    realsense_share = get_package_share_directory("realsense2_camera")
    point_lio_share = get_package_share_directory("point_lio_unilidar")

    slam = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(point_lio_share, "launch", "mapping_utlidar.launch")
        ),
        launch_arguments={
            "rviz": "false",
            "use_sim_time": "false",
        }.items(),
    )
    camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(realsense_share, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "config_file": os.path.join(
                package_share, "config", "realsense_d435.yaml"
            )
        }.items(),
    )
    coarse = Node(
        package="go2_science_perception",
        executable="coarse_target_locator",
        name="coarse_target_locator",
        parameters=[os.path.join(
            package_share, "config", "coarse_target_locator.yaml"
        )],
        output="screen",
    )
    target_to_map = Node(
        package="go2_science_perception",
        executable="camera_target_to_map",
        name="camera_target_to_map",
        parameters=[os.path.join(
            package_share, "config", "camera_target_to_map.yaml"
        )],
        output="screen",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="d435_slam_target_rviz",
        arguments=["-d", os.path.join(
            package_share, "rviz", "d435_slam_target_validation.rviz"
        )],
        output="screen",
    )

    return LaunchDescription([
        GroupAction(actions=[slam], scoped=True),
        GroupAction(actions=[camera], scoped=True),
        static_tf("map_to_camera_init", "map", "camera_init", (0, 0, 0)),
        static_tf("aft_mapped_to_sensor", "aft_mapped", "sensor", (0, 0, 0)),
        static_tf("sensor_to_vehicle", "sensor", "vehicle", (-0.3, 0, 0)),
        coarse,
        target_to_map,
        rviz,
    ])
