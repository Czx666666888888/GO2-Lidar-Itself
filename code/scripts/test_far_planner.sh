#!/bin/bash
# far_planner 离线验证(干净版): 建图链 + TF + far_planner, 触发探索, 检查 /way_point
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
OUTDIR=/home/lch/dog/tmp/far_planner_test2
mkdir -p "$OUTDIR"
BAG=${1:-/home/lch/dog/rosbags/go2_controlled_20260827/obstacle_full_scene_handheld_20260827_01}

PIDS=""
cleanup() { for p in $PIDS; do kill $p 2>/dev/null; done; }

echo "[1] 起建图链 + TF"
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false > /tmp/pio.log 2>&1 & PIDS="$PIDS $!"
ros2 launch terrain_analysis terrain_analysis.launch > /tmp/terrain.log 2>&1 & PIDS="$PIDS $!"
ros2 launch terrain_analysis_ext terrain_analysis_ext.launch > /tmp/terrain_ext.log 2>&1 & PIDS="$PIDS $!"
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map camera_init > /dev/null 2>&1 & PIDS="$PIDS $!"
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 aft_mapped sensor > /dev/null 2>&1 & PIDS="$PIDS $!"
sleep 12

echo "[2] 起 far_planner"
ros2 run far_planner far_planner --ros-args \
  -r /odom_world:=/state_estimation \
  -r /terrain_cloud:=/terrain_map_ext \
  -r /scan_cloud:=/terrain_map \
  -r /terrain_local_cloud:=/registered_scan \
  > "$OUTDIR/far_planner.log" 2>&1 & PIDS="$PIDS $!"
sleep 5

echo "[3] 回放数据(100s) + 触发探索"
timeout 100 ros2 bag play "$BAG" > /dev/null 2>&1 & BAGPID=$!
for i in $(seq 1 10); do
  sleep 9
  ros2 topic pub /update_visibility_graph std_msgs/msg/Bool "{data: true}" --once > /dev/null 2>&1
  echo "  触发 #$i"
done
wait $BAGPID 2>/dev/null

echo "[4] 检查 /way_point"
timeout 4 ros2 topic echo /way_point --once 2>/dev/null | head -10
echo "[5] far_planner 日志(过滤TF警告)"
grep -viE "multicast|interface|Invalid frame|buffer_core|Warning|line 95" "$OUTDIR/far_planner.log" | tail -15

cleanup
echo "完成"
