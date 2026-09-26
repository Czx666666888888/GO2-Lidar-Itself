#!/bin/bash
# ============================================================
# 激活 Go2 ROS2 Humble (RoboStack) 开发环境
# 用法: source scripts/env.sh
# 说明: 系统根分区只读、无 sudo，ROS 2 装在用户工作区 conda 环境里
# ============================================================
export MAMBA_ROOT_PREFIX=/home/lch/dog/tools/mamba
export CONDA_PKGS_DIRS=/home/lch/dog/tools/pkgs
export PATH=/home/lch/dog/tools/bin:$PATH

if [ -x /home/lch/dog/tools/bin/micromamba ] && [ -d /home/lch/dog/ros2/humble ]; then
    eval "$(micromamba shell hook --shell bash)"
    micromamba activate /home/lch/dog/ros2/humble
    echo "[env] ROS2 Humble activated: $CONDA_PREFIX"
    ros2 --help >/dev/null 2>&1 && echo "[env] ros2 CLI ready"
else
    echo "[env] WARNING: micromamba 或 ros2 环境不存在，请先运行 scripts/setup_ros.sh"
fi

# 若已编译过 go2_ws，source 本地 install
if [ -f /home/lch/dog/go2_ws/install/setup.bash ]; then
    source /home/lch/dog/go2_ws/install/setup.bash
    echo "[env] go2_ws/install sourced"
fi
