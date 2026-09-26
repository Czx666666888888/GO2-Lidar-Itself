#!/bin/bash
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
cd /home/lch/dog
exec > /home/lch/dog/tmp/quick_odom.log 2>&1
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false &
sleep 7
timeout 35 ros2 bag play /home/lch/dog/rosbags/go2_controlled_20260824/controlled_stationary_20260824_01 >/dev/null 2>&1 &
sleep 20
echo "=== /state_estimation 位置采样 ==="
timeout 6 ros2 topic echo /state_estimation --field pose.pose.position 2>/dev/null | grep -E "x:|y:|z:" | head -6
