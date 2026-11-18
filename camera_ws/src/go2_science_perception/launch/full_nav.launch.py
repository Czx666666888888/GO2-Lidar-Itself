"""Run the 23:15 navigation planners on FULL's existing map and Point-LIO stream."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("go2_science_perception")) / "config"
    body = yaml.safe_load((config / "body_footprint.yaml").read_text())
    terrain_ext = Path(get_package_share_directory("terrain_analysis_ext"))
    return LaunchDescription([
        Node(package="go2_science_perception", executable="full_nbv_explore",
             name="full_nbv", parameters=[{"escape_enable": False, "stop_on_done": False,
                                           "spin_scan": False, "startup_delay": 3.0}],
             remappings=[("/goal_point", "/mission/nbv_goal"),
                         ("/return_home", "/mission/inactive_return"),
                         ("/escape_reverse", "/mission/inactive_escape"),
                         ("/escape_rotate", "/mission/inactive_rotate"),
                         ("/stop", "/mission/inactive_nbv_stop")],
             output="screen", on_exit=Shutdown()),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(str(config / "full_terrain.xml"))),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(str(terrain_ext / "launch/terrain_analysis_ext.launch")),
            launch_arguments={"checkTerrainConn": "true"}.items()),
        Node(package="far_planner", executable="far_planner", name="far_planner",
             parameters=[str(config / "full_far.yaml"), {"world_frame": "map"}],
             remappings=[("/odom_world", "/base_state_estimation"),
                         ("/terrain_cloud", "/terrain_map_ext"),
                         ("/scan_cloud", "/terrain_map"),
                         ("/terrain_local_cloud", "/registered_scan"),
                         ("/way_point", "/mission/far_way_point")],
             output="screen", on_exit=Shutdown()),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(str(config / "full_local_planner.xml")),
            launch_arguments={"sensorOffsetX": "0.30", "autonomyMode": "true",
                              "autonomySpeed": "0.4", "maxSpeed": "0.4",
                              "vehicleLength": str(body["length"]),
                              "vehicleWidth": str(body["width"]),
                              "obstacleInflate": str(body["hard_clearance"])}.items()),
    ])
