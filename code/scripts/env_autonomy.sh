#!/bin/bash
# ============================================================
# 激活 autonomy_stack_go2 工作区 (Point-LIO + terrain + local_planner)
# 用法: source /home/lch/dog/scripts/env_autonomy.sh
# ============================================================
export HOME=/home/lch/dog/tools/home
export MAMBA_ROOT_PREFIX=/home/lch/dog/tools/mamba
export PATH=/home/lch/dog/tools/bin:$PATH

eval "$(micromamba shell hook --shell bash)"
micromamba activate /home/lch/dog/ros2/humble

# autonomy_stack_go2 工作区
if [ -f /home/lch/dog/thirdparty/autonomy_stack_go2/install/setup.bash ]; then
    source /home/lch/dog/thirdparty/autonomy_stack_go2/install/setup.bash
fi

# CycloneDDS (Go2 通信 / 离线回放一致使用)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 离线回放隔离域(文档 §4 安全前提): 单机回放用
export ROS_DOMAIN_ID=86
# 注意: 必须隔离, 否则会通过网络发现其他 ROS 源导致里程计污染
export ROS_LOCALHOST_ONLY=1
# (已恢复)

# CYCLONEDDS_URI 不用(ROS_LOCALHOST_ONLY 即可)

echo "[autonomy] Point-LIO + terrain + local_planner 环境已激活"
echo "[autonomy] RMW=$RMW_IMPLEMENTATION DOMAIN=$ROS_DOMAIN_ID LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
