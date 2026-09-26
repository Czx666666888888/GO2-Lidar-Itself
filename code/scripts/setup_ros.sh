#!/bin/bash
# ============================================================
# 一键搭建 Go2 ROS2 Humble 开发环境 (无 root 方案, RoboStack)
# 用法: scripts/setup_ros.sh
# 说明: 系统根分区只读且无 sudo 时使用；普通 Ubuntu 也可用，
#       但更推荐直接 apt 安装 ROS2 Humble 官方源。
# ============================================================
set -e
WS=/home/lch/dog
TOOLS=$WS/tools
ENV_DIR=$WS/ros2/humble

mkdir -p "$TOOLS/bin" "$TOOLS/home" "$TOOLS/mamba" "$TOOLS/pkgs"

echo ">>> [1/3] 安装 micromamba"
if [ ! -x "$TOOLS/bin/micromamba" ]; then
  curl -s -o /tmp/mm.tar.bz2 "https://conda.anaconda.org/conda-forge/linux-64/micromamba-2.9.0-0.tar.bz2"
  tar -xjf /tmp/mm.tar.bz2 -C "$TOOLS" bin/micromamba
  chmod +x "$TOOLS/bin/micromamba"
fi
"$TOOLS/bin/micromamba" --version

echo ">>> [2/3] 创建 ROS2 Humble 环境 (约 1GB 下载)"
export HOME="$TOOLS/home" MAMBA_ROOT_PREFIX="$TOOLS/mamba" CONDA_PKGS_DIRS="$TOOLS/pkgs" MAMBA_NO_BANNER=1
if [ ! -x "$ENV_DIR/bin/ros2" ]; then
  "$TOOLS/bin/micromamba" create -y -p "$ENV_DIR" \
    -c https://conda.anaconda.org/robostack-staging \
    -c https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge \
    ros-humble-desktop \
    ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
    ros-humble-pointcloud-to-laserscan ros-humble-teleop-twist-keyboard \
    ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-xacro ros-humble-urdf \
    ros-humble-twist-mux ros-humble-compressed-image-transport \
    ros-humble-foxglove-bridge ros-humble-pcl-ros \
    colcon-common-extensions
fi

echo ">>> [3/3] 完成"
echo "    激活环境:  source $WS/scripts/env.sh"
echo "    编译工作区: scripts/build.sh"
