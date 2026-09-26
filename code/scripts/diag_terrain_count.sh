#!/bin/bash
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
cd /home/lch/dog
LOG=/home/lch/dog/tmp/diag_terrain.log
exec > "$LOG" 2>&1
BAG=/home/lch/dog/rosbags/go2_controlled_20260824/controlled_stationary_20260824_01

ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false &
sleep 8
ros2 launch terrain_analysis terrain_analysis.launch &
sleep 6

# 全程统计 /terrain_map 和 /registered_scan 消息数
timeout 45 ros2 topic echo /terrain_map --field header.stamp > /home/lch/dog/tmp/tm_stamps.txt &
TME=$!
timeout 45 ros2 topic echo /registered_scan --field header.stamp > /home/lch/dog/tmp/rs_stamps.txt &
RSE=$!

ros2 bag play "$BAG" &
BPID=$!

# 分时段采样: 每 6 秒记一次当前消息数
for i in 1 2 3 4 5 6; do
  sleep 6
  echo "t=$((i*6))s terrain=$(wc -l < /home/lch/dog/tmp/tm_stamps.txt 2>/dev/null) registered=$(wc -l < /home/lch/dog/tmp/rs_stamps.txt 2>/dev/null)"
done

wait $BPID 2>/dev/null
wait $TME $RSE 2>/dev/null
echo "FINAL terrain_total=$(wc -l < /home/lch/dog/tmp/tm_stamps.txt) registered_total=$(wc -l < /home/lch/dog/tmp/rs_stamps.txt)"
