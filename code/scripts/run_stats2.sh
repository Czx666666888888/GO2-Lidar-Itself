#!/bin/bash
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
cd /home/lch/dog
exec > /home/lch/dog/tmp/stats.log 2>&1
BAG=/home/lch/dog/rosbags/go2_controlled_20260824/controlled_stationary_20260824_01
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false &
sleep 7
ros2 launch terrain_analysis terrain_analysis.launch &
sleep 6
python3 /home/lch/dog/scripts/stats_terrain.py &
VPID=$!
sleep 3
ros2 bag play "$BAG" >/dev/null 2>&1 &
wait $VPID 2>/dev/null
echo "STATS_DONE"
