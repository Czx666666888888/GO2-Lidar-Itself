#!/bin/bash
# ============================================================
# Go2 环境/运行状态一键检查 (无需手动 source)
# 用法: bash scripts/check.sh
# ============================================================
source "$(dirname "$0")/env.sh" 2>/dev/null

echo ""
echo "========== 1. ROS2 环境 =========="
if command -v ros2 >/dev/null 2>&1; then
    echo "✓ ros2 可用: $(command -v ros2)"
else
    echo "✗ ros2 未找到! (先: source /home/lch/dog/scripts/env.sh)"
    exit 1
fi

echo ""
echo "========== 2. 工作区包 =========="
for p in go2_slam_nav go2_robot_sdk fast_lio livox_ros_driver2 unitree_go; do
    if ros2 pkg prefix "$p" >/dev/null 2>&1; then
        echo "✓ $p"
    else
        echo "✗ $p 未安装"
    fi
done

echo ""
echo "========== 3. 运行中的节点 =========="
NODES=$(timeout 6 ros2 node list 2>/dev/null)
if [ -n "$NODES" ]; then
    echo "$NODES" | head -20
else
    echo "(暂无节点运行 — 先启动: ROBOT_IP=... ros2 launch go2_slam_nav go2_mapping.launch.py)"
fi

echo ""
echo "========== 4. 关键话题 =========="
TOPICS=$(timeout 6 ros2 topic list 2>/dev/null)
for t in /odom /joint_states /scan /map /imu; do
    if echo "$TOPICS" | grep -qx "$t"; then
        echo "✓ $t"
    else
        echo "- $t (未发布)"
    fi
done
echo ""
echo "完成。真机联调时看到 /odom 有数据 = WebRTC 链路已通"
