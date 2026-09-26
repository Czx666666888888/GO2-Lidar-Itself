#!/bin/bash
# ============================================================
# WP0 离线复现: 静止包 → Point-LIO + terrain_analysis 链
# 用法: bash scripts/replay_stationary.sh
# 输出: /home/lch/dog/tmp/replay_stationary.log
# ============================================================
LOG=/home/lch/dog/tmp/replay_stationary.log
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
exec > "$LOG" 2>&1
echo "===== WP0 静止包离线复现 开始 $(date) ====="

BAG=/home/lch/dog/rosbags/go2_controlled_20260824/controlled_stationary_20260824_01

echo ">>> [1/4] 启动 Point-LIO (含 transform_sensors)"
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false &
LPID=$!
sleep 8

echo ">>> [2/4] 启动 terrain_analysis"
ros2 launch terrain_analysis terrain_analysis.launch &
TPID=$!
sleep 6

echo ">>> 已启动节点:"
ros2 node list 2>&1 | head -10

echo ">>> [3/4] 回放静止包 (36s)"
ros2 bag play "$BAG" &
BPID=$!

for i in 1 2 3 4 5 6; do
  sleep 5
  echo "--- 播放 $((i*5))s ---"
  echo -n "  /state_estimation: "; timeout 3 ros2 topic hz /state_estimation 2>/dev/null | grep "average rate" | head -1 || echo "无"
  echo -n "  /terrain_map: "; timeout 3 ros2 topic hz /terrain_map 2>/dev/null | grep "average rate" | head -1 || echo "无"
done

echo ">>> [4/4] 结果抽样:"
echo -n "  /state_estimation 一条消息: "
timeout 4 ros2 topic echo /state_estimation --once 2>&1 | grep -m1 "pose:" && echo "OK" || echo "无数据"
echo -n "  /registered_scan 一条消息: "
timeout 4 ros2 topic echo /registered_scan --once 2>&1 | grep -m1 "frame_id" && echo "OK" || echo "无数据"
echo -n "  /terrain_map 一条消息: "
timeout 4 ros2 topic echo /terrain_map --once 2>&1 | grep -m1 "frame_id" && echo "OK" || echo "无数据"

# 等 bag 放完再收尾
wait $BPID 2>/dev/null
kill $TPID $LPID 2>/dev/null
sleep 2
echo "===== 复现结束 $(date) ====="
