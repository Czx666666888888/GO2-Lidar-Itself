#!/bin/bash
# 一键: 连接 Go2 边跑边建图
# 用法: ROBOT_IP=192.168.123.161 CONN_TYPE=webrtc scripts/run_mapping.sh
set -e
source "$(dirname "$0")/env.sh"

if [ -z "${ROBOT_IP:-}" ]; then
  echo "错误: 请设置 ROBOT_IP 环境变量 (狗在 Wi-Fi 模式下的 IP)"
  echo "  例: ROBOT_IP=192.168.123.161 scripts/run_mapping.sh"
  exit 1
fi
export ROBOT_IP CONN_TYPE="${CONN_TYPE:-webrtc}"

echo ">>> 连接 ${ROBOT_IP} (${CONN_TYPE}), 开始建图..."
echo ">>> 遥控: 手柄或 ros2 run teleop_twist_keyboard teleop_twist_keyboard"
ros2 launch go2_slam_nav go2_mapping.launch.py "$@"
