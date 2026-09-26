#!/bin/bash
# 方案B: 合成障碍场景(精简链: 无 terrainAnalysis, localPlanner 用激光)
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
cd /home/lch/dog
exec > /home/lch/dog/tmp/synth.log 2>&1
INPUT="$1"; SCEN="${2:-1}"
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false &
sleep 7
ros2 run local_planner localPlanner --ros-args --params-file /home/lch/dog/scripts/local_planner_dry_run.yaml &
sleep 4
timeout 50 ros2 bag play "$INPUT" >/dev/null 2>&1 &
sleep 22
python3 /home/lch/dog/scripts/synth_obstacle.py "$SCEN"
wait 2>/dev/null
echo "SYNTH_SHELL_DONE"
