#!/bin/bash
# 录制完整建图导航链 rosbag, 按 标签+时间戳 自动命名(每次唯一, 不覆盖)
# 用法:
#   bash scripts/record_full_chain.sh            # full_chain_session_20260830_143000
#   bash scripts/record_full_chain.sh test1      # full_chain_test1_20260830_143000
source /home/lch/dog/go2_ws/setup_realtime.sh

TAG=${1:-session}
TS=$(date +%Y%m%d_%H%M%S)
OUT=/home/lch/dog/rosbags/full_chain_${TAG}_${TS}

TOPICS="/utlidar/cloud /utlidar/imu \
/state_estimation /base_state_estimation /registered_scan /terrain_map \
/goal_point /way_point /path \
/cmd_vel_raw /api/sport/request /lf/sportmodestate /far_reach_goal_status"

echo "=============================================="
echo "录制完整链 rosbag"
echo "标签: $TAG | 时间: $TS"
echo "输出: $OUT"
echo "话题: 传感器+建图+规划+速度+狗状态 共11个"
echo "=============================================="

ros2 bag record -o "$OUT" $TOPICS
