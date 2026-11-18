"""Standalone navigation launch using the 2026-11-18 planner profiles."""
import os
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).parent
    stack = root / "autonomy_stack"
    footprint = yaml.safe_load((root / "body_footprint.yaml").read_text())

    def include(path, **args):
        return IncludeLaunchDescription(AnyLaunchDescriptionSource(str(path)),
                                        launch_arguments=args.items())

    def script(path, name, params=None):
        return Node(executable="/usr/bin/python3", arguments=[str(path)], name=name,
                    parameters=[params or {}], output="screen", on_exit=Shutdown())

    return LaunchDescription([
        script(stack / "transform/transform_everything.py", "transform_everything"),
        include(stack / "point_lio/mapping_utlidar.launch", rviz="false"),
        include(stack / "terrain/terrain_analysis.launch"),
        include(Path(get_package_share_directory("terrain_analysis_ext")) /
                "launch/terrain_analysis_ext.launch", checkTerrainConn="true"),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["0", "0", "0", "0", "0", "0", "map", "camera_init"]),
        Node(package="go2_keyboard_teleop", executable="base_odom_node", on_exit=Shutdown()),
        Node(package="far_planner", executable="far_planner", name="far_planner",
             parameters=[str(stack / "far_planner/default.yaml")], output="screen",
             remappings=[("/odom_world", "/base_state_estimation"), ("/terrain_cloud", "/terrain_map_ext"),
                         ("/scan_cloud", "/terrain_map"), ("/terrain_local_cloud", "/registered_scan"),
                         ("/way_point", "/navigation_test/far_waypoint")], on_exit=Shutdown()),
        include(stack / "local_planner/local_planner.launch", autonomyMode="true",
                autonomySpeed="0.4", maxSpeed="0.4", sensorOffsetX="0.3",
                vehicleLength=str(footprint["length"]), vehicleWidth=str(footprint["width"]),
                obstacleInflate=str(footprint["hard_clearance"])),
        script(root / "nodes/obstacle_memory_node.py", "obstacle_memory_node"),
        script(root / "nodes/nbv_explore_node.py", "nbv_explore_node",
               {"startup_delay": 2., "return_after": 60., "no_progress_timeout": 20.}),
        script(root / "nodes/safety_gate.py", "navigation_test_gate",
               {"dry_run": os.environ.get("NAV_EXECUTE") != "1",
                "cmd_vel_topic": "/navigation_test/cmd_vel", "arm_topic": "/navigation_test/arm",
                "stop_topic": "/navigation_test/stop", "max_speed": .4, "min_speed": .30, "max_yaw_rate": .70}),
    ])
