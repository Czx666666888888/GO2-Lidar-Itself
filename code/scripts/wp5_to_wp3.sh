#!/bin/bash
# ①串闭环: 回放正方形包, 在轨迹后段发布 WP5 最优观察点, 验证 localPlanner 规划
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
cd /home/lch/dog
exec > /home/lch/dog/tmp/wp5_to_wp3.log 2>&1
INPUT="$1"; GX="${2:--3.07}"; GY="${3:-1.97}"
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false &
sleep 7
ros2 launch terrain_analysis terrain_analysis.launch &
sleep 6
ros2 run local_planner localPlanner --ros-args --params-file /home/lch/dog/scripts/local_planner_dry_run.yaml &
sleep 4
timeout 130 ros2 bag play "$INPUT" >/dev/null 2>&1 &
sleep 100   # 等轨迹走到后段(接近 home)
python3 /home/lch/dog/scripts/wp5_to_wp3.py "$GX" "$GY"
wait 2>/dev/null
echo "SHELL_DONE"
