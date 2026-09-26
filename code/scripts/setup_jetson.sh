#!/bin/bash
# ============================================================
# Go2 载机 Jetson (aarch64) 部署脚本
# 在 Jetson 上运行: bash scripts/setup_jetson.sh
# 前提: Ubuntu 22.04 (JetPack 6) 或 20.04 (JetPack 5), 有 root 更好
#
# 方案: RoboStack 提供 linux-aarch64 的 ROS2 Humble 包,
#       无需在本机源码编译 ROS 本体; 本工作区源码在 Jetson 上 colcon 编译。
# ============================================================
set -e
WS="$(cd "$(dirname "$0")/.." && pwd)"
TOOLS=$WS/tools
ENV_DIR=$WS/ros2/humble
ARCH="$(uname -m)"
echo ">>> Jetson arch: $ARCH"

if [ "$ARCH" != "aarch64" ]; then
  echo "错误: 本脚本应在 Go2 的 Jetson (aarch64) 上运行"
  exit 1
fi

mkdir -p "$TOOLS/bin" "$TOOLS/home" "$TOOLS/mamba" "$TOOLS/pkgs"

echo ">>> [1/4] 安装 micromamba (aarch64)"
if [ ! -x "$TOOLS/bin/micromamba" ]; then
  curl -s -o /tmp/mm.tar.bz2 \
    "https://conda.anaconda.org/conda-forge/linux-aarch64/micromamba-2.9.0-0.tar.bz2"
  tar -xjf /tmp/mm.tar.bz2 -C "$TOOLS" bin/micromamba
  chmod +x "$TOOLS/bin/micromamba"
fi
"$TOOLS/bin/micromamba" --version

echo ">>> [2/4] 创建 ROS2 Humble 环境 (aarch64, 约 1GB)"
export HOME="$TOOLS/home" MAMBA_ROOT_PREFIX="$TOOLS/mamba" CONDA_PKGS_DIRS="$TOOLS/pkgs" MAMBA_NO_BANNER=1
if [ ! -x "$ENV_DIR/bin/ros2" ]; then
  "$TOOLS/bin/micromamba" create -y -p "$ENV_DIR" \
    -c https://conda.anaconda.org/robostack-staging \
    -c https://conda.anaconda.org/conda-forge \
    ros-humble-desktop \
    ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
    ros-humble-pointcloud-to-laserscan ros-humble-teleop-twist-keyboard \
    ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-xacro ros-humble-urdf \
    ros-humble-twist-mux ros-humble-compressed-image-transport \
    ros-humble-foxglove-bridge ros-humble-pcl-ros \
    ros-humble-rosidl-generator-dds-idl \
    colcon-common-extensions
fi

echo ">>> [3/4] 编译 Go2 工作区"
cd "$WS/go2_ws"
export PATH="$TOOLS/bin:$PATH"
eval "$("$TOOLS/bin/micromamba" shell hook --shell bash)"
micromamba activate "$ENV_DIR"
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$ENV_DIR"

echo ">>> [4/4] 完成!"
echo "    每次使用前: source $WS/scripts/env.sh"
echo "    建图:        ROBOT_IP=127.0.0.1 ros2 launch go2_slam_nav go2_mapping.launch.py"
echo "    注: Jetson 上雷达已直连, 也可直接 ros2 launch go2_slam_nav fastlio_mapping.launch.py"
