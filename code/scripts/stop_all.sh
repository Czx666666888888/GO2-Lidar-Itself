#!/bin/bash
# ============================================================
# 停止所有 Go2 相关 ROS 进程 (用于重启/复位)
# 用法: bash scripts/stop_all.sh
# ============================================================
echo ">>> 停止所有 Go2 ROS 进程..."

pkill -f "go2_driver_node" 2>/dev/null
pkill -f "go2_robot_state_publisher" 2>/dev/null
pkill -f "pointcloud_to_laserscan_node" 2>/dev/null
pkill -f "lidar_to_pointcloud" 2>/dev/null
pkill -f "pointcloud_aggregator" 2>/dev/null
pkill -f "async_slam_toolbox" 2>/dev/null
pkill -f "slam_toolbox" 2>/dev/null
pkill -f "tts_node" 2>/dev/null
pkill -f "twist_mux" 2>/dev/null
pkill -f "joy_node" 2>/dev/null
pkill -f "teleop_node" 2>/dev/null
pkill -f "rviz2" 2>/dev/null
pkill -f "foxglove_bridge" 2>/dev/null
pkill -f "fastlio_mapping" 2>/dev/null
pkill -f "livox_ros_driver2_node" 2>/dev/null
pkill -f "controller_server" 2>/dev/null
pkill -f "planner_server" 2>/dev/null
pkill -f "behavior_server" 2>/dev/null
pkill -f "bt_navigator" 2>/dev/null
pkill -f "waypoint_follower" 2>/dev/null
pkill -f "velocity_smoother" 2>/dev/null
pkill -f "map_server" 2>/dev/null
pkill -f "lifecycle_manager" 2>/dev/null

sleep 2
echo ">>> 完成。残留进程检查:"
ps aux 2>/dev/null | grep -E "go2_|slam_toolbox|rviz2|fastlio|livox_ros" | grep -v grep | grep -v "stop_all" | head -5 || echo "(无残留)"
