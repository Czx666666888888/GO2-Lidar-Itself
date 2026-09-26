#!/bin/bash
# 一键: 加载地图并导航
# 用法: ROBOT_IP=192.168.123.161 scripts/run_navigation.sh /path/to/map.yaml
set -e
source "$(dirname "$0")/env.sh"

if [ -z "${ROBOT_IP:-}" ]; then
  echo "错误: 请设置 ROBOT_IP 环境变量"
  exit 1
fi
MAP="${1:-}"
export ROBOT_IP CONN_TYPE="${CONN_TYPE:-webrtc}"

if [ -n "$MAP" ]; then
  [ -f "$MAP" ] || { echo "地图文件不存在: $MAP"; exit 1; }
  echo ">>> 加载地图 $MAP 并导航"
  ros2 launch go2_slam_nav go2_navigation.launch.py map:="$MAP"
else
  echo ">>> 无静态地图, 依赖在线 /map 导航 (请先跑建图)"
  ros2 launch go2_slam_nav go2_navigation.launch.py
fi
