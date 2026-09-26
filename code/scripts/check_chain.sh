#!/bin/bash
# 建图链逐环检查: 狗传感器 → transform → Point-LIO → terrain
# 找出哪一环断了(话题没数据/节点没起来)
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
GO2_IFACE="${GO2_IFACE:-enp12s0}"
source "$ROOT/code/scripts/env_go2.sh" "$GO2_IFACE" || exit 1

echo "================ 建图链检查 ================"

echo ""
echo "[0] 节点状态"
ros2 node list 2>/dev/null | grep -E "laserMapping|terrain|transform|far_planner|localPlanner|pathFollower" || echo "  ⚠️ 没找到核心节点(导航链没起?)"

check_topic() {
    local topic=$1
    local name=$2
    echo -n "  $name ($topic): "
    local rate=$(timeout 8 ros2 topic hz "$topic" --window 10 2>/dev/null | grep "average rate" | head -1)
    if [ -n "$rate" ]; then
        echo "✅ $rate"
    else
        echo "❌ 无数据"
    fi
}

echo ""
echo "[1] 狗传感器(最上游)"
check_topic /utlidar/cloud "激光雷达"
check_topic /utlidar/imu "IMU"

echo ""
echo "[2] transform_sensors(坐标变换)"
check_topic /utlidar/transformed_cloud "变换后点云"
check_topic /utlidar/transformed_imu "变换后IMU"

echo ""
echo "[3] Point-LIO(建图核心)"
check_topic /state_estimation "位姿估计(雷达)"
check_topic /base_state_estimation "位姿估计(机身中心)"
check_topic /registered_scan "注册点云"

echo ""
echo "[4] terrain(地形图)"
check_topic /terrain_map "地形图"
check_topic /terrain_map_ext "扩展地形"

echo ""
echo "[5] 导航链(下游)"
check_topic /way_point "far_planner目标"
check_topic /far_reach_goal_status "far到达确认"
check_topic /path "local_planner路径"
check_topic /cmd_vel_raw "pathFollower速度"

echo ""
echo "================ 断点判断 ================"
echo "从上往下看, 第一个 ❌ 就是断点:"
echo "  [1]❌ = 狗没连上(网线/网络)"
echo "  [2]❌ = transform_sensors 没起"
echo "  [3]❌ = Point-LIO 没起或没收到变换数据"
echo "  [4]❌ = terrain 没起或没收到注册点云"
echo "  [5]❌ = 下游没起或上游没数据"
