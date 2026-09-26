#!/bin/bash
# ============================================================
# 停止 Go2 自主导航链的所有节点 + 让狗停止 (急停)
# 用法: bash scripts/nav_stop.sh
# ============================================================
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
GO2_IFACE="${GO2_IFACE:-enp12s0}"
source "$ROOT/code/scripts/env_go2.sh" "$GO2_IFACE" || exit 1

echo ">>> 停止导航链并让狗停下..."

# 1. 先直接发 StopMove (1003), 确保狗立刻停 (无论安全门在不在)
ros2 topic pub /api/sport/request unitree_api/msg/Request \
    "{header: {identity: {api_id: 1003}}}" --once >/dev/null 2>&1 \
    && echo "    ✓ 已发 StopMove(1003)"

# 2. 按可执行名精确清理所有导航节点
pkill -INT -f "pointlio_mapping"     2>/dev/null
pkill -INT -f "transform_everything" 2>/dev/null
pkill -INT -f "localPlanner"         2>/dev/null
pkill -INT -f "pathFollower"         2>/dev/null
pkill -INT -f "terrainAnalysis"      2>/dev/null
pkill -INT -f "terrainAnalysisExt"   2>/dev/null
pkill -INT -f "far_planner"          2>/dev/null
pkill -INT -f "graph_decoder"        2>/dev/null
pkill -INT -f "boundary_handler"     2>/dev/null
pkill -INT -f "go2_safety_gate"      2>/dev/null
pkill -INT -f "wp5_explore_node"     2>/dev/null
pkill -INT -f "base_odom_node"       2>/dev/null
pkill -INT -f "rviz2.*far_planner/rviz/default.rviz" 2>/dev/null

# 3. 兜底 SIGTERM (有些节点忽略 SIGINT)
sleep 2
pkill -TERM -f "pointlio_mapping\|localPlanner\|pathFollower\|terrainAnalysis\|far_planner\|go2_safety_gate\|wp5_explore_node\|transform_everything\|base_odom_node\|rviz2.*far_planner/rviz/default.rviz" 2>/dev/null

sleep 1
echo ">>> 残留节点检查:"
ps aux 2>/dev/null | grep -E "pointlio_mapping|localPlanner|pathFollower|terrainAnalysis|far_planner|go2_safety_gate|wp5_explore_node|transform_everything|base_odom_node" | grep -v grep | head -10 || true
echo ">>> 完成。"
