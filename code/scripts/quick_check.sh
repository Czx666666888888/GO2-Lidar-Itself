#!/bin/bash
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
cd /home/lch/dog
exec > /home/lch/dog/tmp/quick_check.log 2>&1
BAG="$1"
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false &
sleep 7
ros2 launch terrain_analysis terrain_analysis.launch &
sleep 6
timeout 45 ros2 bag play "$BAG" >/dev/null 2>&1 &
sleep 30
echo "=== 30s 时刻速率 ==="
for t in /state_estimation /registered_scan /terrain_map; do
  echo -n "$t: "; timeout 4 ros2 topic hz $t 2>/dev/null | grep -m1 "average rate" || echo 无
done
