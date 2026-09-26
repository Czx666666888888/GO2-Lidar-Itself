#!/bin/bash
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
cd /home/lch/dog
exec > /home/lch/dog/tmp/wp3_unreachable.log 2>&1
INPUT="$1"
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false &
sleep 7
ros2 launch terrain_analysis terrain_analysis.launch &
sleep 6
ros2 run local_planner localPlanner --ros-args --params-file /home/lch/dog/scripts/local_planner_dry_run.yaml &
sleep 4
timeout 55 ros2 bag play "$INPUT" >/dev/null 2>&1 &
python3 /home/lch/dog/scripts/wp3_unreachable.py
wait 2>/dev/null
echo "UNREACHABLE_SHELL_DONE"
