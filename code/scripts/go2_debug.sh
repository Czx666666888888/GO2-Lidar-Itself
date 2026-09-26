#!/bin/bash
# 真机调试版: 一键启动自主导航 + 记录建图数据 + 导航决策路径, 键盘控制启停
#
# 启动内容:
#   1. 完整自主导航链(Point-LIO + terrain + far_planner + local_planner + pathFollower)
#   2. rosbag 记录: 建图数据(传感器) + 导航决策(探索目标/路径/速度)
#   3. 键盘控制: s=触发探索, e=急停, q=退出
#
# 用法:
#   bash go2_debug.sh              # 默认, 录包自动开始
#   bash go2_debug.sh 300          # 录包最长 300 秒
#
source /home/lch/dog/go2_ws/setup_realtime.sh

REC_TIME=${1:-0}
TAG=debug_$(date +%Y%m%d_%H%M%S)
BAGDIR=/home/lch/dog/rosbags/nav_$TAG

echo "======================================"
echo "真机调试版启动"
echo "导航链: Point-LIO + terrain + far_planner + local_planner + pathFollower"
echo "录包: $BAGDIR"
echo "键盘: s/空格=触发探索 | e=急停 | q=退出"
echo "======================================"

# 1. 启动完整自主导航链
ros2 launch vehicle_simulator system_real_robot_with_route_planner.launch > /tmp/nav_debug.log 2>&1 &
NAVPID=$!
sleep 12

# 2. 录包(建图数据 + 导航决策话题)
#    建图数据: 传感器原始
#    导航决策: 位姿/地形/探索目标/路径/速度/边界/到达标志
REC_TOPICS="/utlidar/imu /utlidar/cloud /utlidar/robot_odom /lf/sportmodestate \
  /state_estimation /registered_scan /terrain_map /terrain_map_ext \
  /way_point /path /cmd_vel /navigation_boundary /far_reach_goal_status"
if [ "$REC_TIME" -gt 0 ]; then
    timeout "$REC_TIME" ros2 bag record $REC_TOPICS -o "$BAGDIR" > /tmp/rec_debug.log 2>&1 &
else
    ros2 bag record $REC_TOPICS -o "$BAGDIR" > /tmp/rec_debug.log 2>&1 &
fi
RECPID=$!

# 3. 键盘控制启停(前台, s=触发探索 e=急停 q=退出)
ros2 run go2_keyboard_teleop go2_debug_control

# 退出清理
echo "===== 调试结束, 清理 ====="
kill $RECPID 2>/dev/null
kill $NAVPID 2>/dev/null
sleep 2
echo "录包保存在: $BAGDIR"
ls -la "$BAGDIR" 2>/dev/null
