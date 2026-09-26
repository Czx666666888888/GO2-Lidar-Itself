#!/bin/bash
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
cd /home/lch/dog
LOG=/home/lch/dog/tmp/diag_hz.log
exec > "$LOG" 2>&1
BAG=/home/lch/dog/rosbags/go2_controlled_20260824/controlled_stationary_20260824_01
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false &
sleep 7
ros2 launch terrain_analysis terrain_analysis.launch &
sleep 6
ros2 bag play "$BAG" >/dev/null 2>&1 &
BPID=$!
sleep 20
echo "=== 播放中 20s 时刻的速率 ==="
echo -n "/state_estimation: "; timeout 4 ros2 topic hz /state_estimation 2>/dev/null | grep -m1 "average rate" || echo 无
echo -n "/registered_scan: "; timeout 4 ros2 topic hz /registered_scan 2>/dev/null | grep -m1 "average rate" || echo 无
echo -n "/terrain_map: "; timeout 4 ros2 topic hz /terrain_map 2>/dev/null | grep -m1 "average rate" || echo 无
wait $BPID 2>/dev/null
