#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="$ROOT/deploy_ws"

source /opt/ros/humble/setup.bash

missing=()
for pkg in rosidl_generator_dds_idl rmw_cyclonedds_cpp tf_transformations; do
  if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    missing+=("$pkg")
  fi
done
if ((${#missing[@]})); then
  echo "缺少 ROS 包: ${missing[*]}" >&2
  echo "请先运行:" >&2
  echo "  sudo apt-get update" >&2
  echo "  sudo apt-get install -y ros-humble-rosidl-generator-dds-idl ros-humble-rmw-cyclonedds-cpp ros-humble-tf-transformations" >&2
  exit 2
fi

mkdir -p "$WS/src"
ln -sfn "$ROOT/code/autonomy_stack_go2_src" "$WS/src/autonomy_stack_go2_src"
ln -sfn "$ROOT/code/go2_keyboard_teleop" "$WS/src/go2_keyboard_teleop"

cd "$WS"
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
  --parallel-workers "${BUILD_WORKERS:-6}"

echo "部署完成: $WS"
echo "下一步: source '$ROOT/code/scripts/env_go2.sh' enp12s0"
