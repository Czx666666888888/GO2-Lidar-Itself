#!/bin/bash
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
cd /home/lch/dog
LOG=/home/lch/dog/tmp/diag_3x.log
exec > "$LOG" 2>&1
BAG=/home/lch/dog/rosbags/go2_controlled_20260824/controlled_stationary_20260824_01
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false &
sleep 7
for t in /utlidar/cloud /utlidar/transformed_cloud /registered_scan; do
  fname=$(echo $t | tr '/' '_')
  timeout 45 ros2 topic echo $t --field header.stamp > /home/lch/dog/tmp/x_$fname.txt 2>/dev/null &
done
sleep 2
ros2 bag play "$BAG" >/dev/null 2>&1 &
BPID=$!
wait $BPID 2>/dev/null
sleep 4
echo "=== 计数结果 (36s 静止包, 输入 cloud=542) ==="
for t in /utlidar/cloud /utlidar/transformed_cloud /registered_scan; do
  fname=$(echo $t | tr '/' '_')
  n=$(wc -l < /home/lch/dog/tmp/x_$fname.txt 2>/dev/null)
  echo "$t : $n 条"
done
