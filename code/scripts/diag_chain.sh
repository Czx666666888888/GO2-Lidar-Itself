#!/bin/bash
# 全链路消息计数: 定位 3x 发生在哪一环
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
cd /home/lch/dog
LOG=/home/lch/dog/tmp/diag_chain.log
exec > "$LOG" 2>&1
BAG=/home/lch/dog/rosbags/go2_controlled_20260824/controlled_stationary_20260824_01

ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false &
sleep 8
ros2 launch terrain_analysis terrain_analysis.launch &
sleep 6

for t in /utlidar/cloud /utlidar/transformed_cloud /registered_scan /state_estimation /terrain_map; do
  fname=$(echo $t | tr '/' '_')
  timeout 48 ros2 topic echo $t --field header.stamp > /home/lch/dog/tmp/cnt_$fname.txt &
done
sleep 1

ros2 bag play "$BAG" &
BPID=$!

wait $BPID 2>/dev/null
sleep 5
echo "=== 全链路消息计数(36s 静止包) ==="
for t in /utlidar/cloud /utlidar/transformed_cloud /registered_scan /state_estimation /terrain_map; do
  fname=$(echo $t | tr '/' '_')
  n=$(wc -l < /home/lch/dog/tmp/cnt_$fname.txt 2>/dev/null)
  rate=$(python3 -c "print(f'{$n/36:.1f}')")
  echo "$t : $n 条 (~$rate Hz)"
done
pkill -f "ros2 topic echo" 2>/dev/null
pkill -f "laserMapping" 2>/dev/null
pkill -f "terrainAnalysis" 2>/dev/null
pkill -f "transform_everything" 2>/dev/null
echo "=== 完成 ==="
