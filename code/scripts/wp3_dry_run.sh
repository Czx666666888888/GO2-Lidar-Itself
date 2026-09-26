#!/bin/bash
# WP3: local_planner 离线 dry-run (只起 localPlanner, 不起 pathFollower)
# 用法: bash wp3_dry_run.sh <输入bag> <目标x> <目标y> [duration秒]
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
cd /home/lch/dog
INPUT="$1"; GX="${2:-2.0}"; GY="${3:-0.0}"; DUR="${4:-60}"
exec > /home/lch/dog/tmp/wp3.log 2>&1

echo ">>> WP3 dry-run: bag=$INPUT goal=($GX,$GY) dur=$DUR"
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false &
sleep 7
ros2 launch terrain_analysis terrain_analysis.launch &
sleep 6
# 只起 localPlanner (dry-run, 无 pathFollower)
ros2 run local_planner localPlanner --ros-args --params-file /home/lch/dog/scripts/local_planner_dry_run.yaml &
LPPID=$!
sleep 4

# 回放数据
timeout "$DUR" ros2 bag play "$INPUT" >/dev/null 2>&1 &
BGPID=$!

# 等里程计/地形起来后发目标点(约 25s 后)
sleep 25
echo ">>> 发布目标点 ($GX, $GY) 到 /way_point"
ros2 topic pub -1 /way_point geometry_msgs/msg/PointStamped "{header: {frame_id: 'camera_init'}, point: {x: $GX, y: $GY, z: 0.0}}" 2>&1 | head -2

# 观察 /path 输出
sleep 8
echo ">>> /path 输出:"
timeout 4 ros2 topic echo /path --once 2>/dev/null | grep -E "header|frame_id|pose:" | head -8 || echo "  /path 无输出"

echo ">>> /free_paths 输出:"
timeout 4 ros2 topic echo /free_paths --once 2>/dev/null | grep -cE "frame_id|data" || echo "  无"

wait $BGPID 2>/dev/null
echo "WP3_DONE"
