#!/bin/bash
# 用 rosbag 原始传感器数据跑完整导航链(建图+规划+跟踪+WP5选点), 收集导航输出
# 用法: bash scripts/replay_nav_chain.sh <bag目录> [输出目录]
BAG=${1:?用法: bash replay_nav_chain.sh <bag目录> [输出目录]}
OUT=${2:-/home/lch/dog/tmp/nav_chain_out}

source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
source /home/lch/dog/go2_ws/install/setup.bash 2>/dev/null   # 加 go2 工作区(WP5/base_odom)
export ROS_DOMAIN_ID=87   # 换 domain 避开 loopback participant index 问题

BAG_DUR=$(ros2 bag info "$BAG" 2>/dev/null | awk '/Duration:/{printf "%.0f", $2}')
[ -z "$BAG_DUR" ] && BAG_DUR=140
COLLECT_DUR=$(( BAG_DUR + 30 ))

mkdir -p "$OUT"
rm -f "$OUT"/*.npz 2>/dev/null

echo "=============================================="
echo "离线导航链: $BAG (${BAG_DUR}s) → $OUT"
echo "=============================================="

# 0.5 机身中心里程计转换(FAR 现在需要 /base_state_estimation)
ros2 run go2_keyboard_teleop base_odom_node \
    > "$OUT/base_odom.log" 2>&1 &
BASEPID=$!
sleep 2

# 1. 完整导航链(Point-LIO + terrain + far/local planner + pathFollower)
ros2 launch vehicle_simulator system_real_robot_with_route_planner.launch \
    > "$OUT/nav_chain.log" 2>&1 &
NAVPID=$!
echo "[1] 导航链已启动, 等初始化 15s..."
sleep 15

# 2. WP5 在线探索(自动选点发 /goal_point)
ros2 run go2_keyboard_teleop wp5_explore_node \
    > "$OUT/wp5.log" 2>&1 &
WP5PID=$!
sleep 3

# 3. 触发 far_planner
ros2 topic pub /update_visibility_graph std_msgs/Bool "{data: true}" --once >/dev/null 2>&1

# 4. 采集器(订阅 goal/waypoint/path/cmd_vel/traj)
python3 /home/lch/dog/scripts/collect_nav_chain.py "$COLLECT_DUR" "$OUT" \
    > "$OUT/collect.log" 2>&1 &
CPID=$!
sleep 3

# 5. 只回放原始传感器(避免和 bag 里已录话题冲突)
echo "[2] 回放原始传感器(/utlidar/cloud + /utlidar/imu)..."
ros2 bag play "$BAG" --topics /utlidar/cloud /utlidar/imu > "$OUT/play.log" 2>&1
echo "[3] 回放完成"

# 6. 等采集器
wait $CPID 2>/dev/null

# 7. 清理
kill $NAVPID $WP5PID $BASEPID 2>/dev/null
pkill -f pointlio_mapping 2>/dev/null
pkill -f transform_everything 2>/dev/null
pkill -f terrainAnalysis 2>/dev/null
pkill -f far_planner 2>/dev/null
pkill -f localPlanner 2>/dev/null
pkill -f pathFollower 2>/dev/null
pkill -f wp5_explore_node 2>/dev/null
pkill -f base_odom_node 2>/dev/null
sleep 2

echo "=============================================="
echo "导航链输出摘要:"
cat "$OUT/collect.log" 2>/dev/null
echo "=============================================="
