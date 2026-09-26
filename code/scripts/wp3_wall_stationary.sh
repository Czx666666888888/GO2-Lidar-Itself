#!/bin/bash
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
cd /home/lch/dog
exec > /home/lch/dog/tmp/wp3_wall.log 2>&1
INPUT=/home/lch/dog/rosbags/go2_controlled_20260824/controlled_stationary_20260824_01
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false &
sleep 7
ros2 launch terrain_analysis terrain_analysis.launch &
sleep 6
ros2 run local_planner localPlanner --ros-args --params-file /home/lch/dog/scripts/local_planner_dry_run.yaml &
sleep 4
timeout 40 ros2 bag play "$INPUT" >/dev/null 2>&1 &
sleep 18
# 先发可达目标
ros2 topic pub -1 /way_point geometry_msgs/msg/PointStamped "{header: {frame_id: 'camera_init'}, point: {x: 1.5, y: 0.0, z: 0.0}}" >/dev/null 2>&1
sleep 2
python3 /home/lch/dog/scripts/wp3_wall.py
wait 2>/dev/null
echo "WALL_STATIONARY_DONE"
