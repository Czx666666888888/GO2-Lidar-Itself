#!/bin/bash
# 端到端建图测试: 启动建图链 + 回放采集数据 + 监听 terrain_map
set -u
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1

REPLAY_TIME=${1:-30}

echo "===== [1] 启动建图链 ====="
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false > /tmp/mapping.log 2>&1 &
MPID=$!
ros2 launch terrain_analysis terrain_analysis.launch > /tmp/terrain.log 2>&1 &
TPID=$!

echo "等待建图链就绪(10s)..."
sleep 10

echo "===== [2] 回放 ${REPLAY_TIME}s 数据 ====="
python3 /home/lch/dog/scripts/replay_dds_data.py --max_time "$REPLAY_TIME" > /tmp/replay.log 2>&1 &
RPID=$!

# 等回放进行到一半时监听 terrain_map
sleep $((REPLAY_TIME / 2))
echo "===== [3] terrain_map 输出(监听 10s) ====="
timeout 10 ros2 topic hz /terrain_map 2>&1 | head -5
timeout 3 ros2 topic echo /terrain_map --once 2>/dev/null | head -8

echo "===== [4] 等待回放完成 ====="
wait $RPID 2>/dev/null

echo "===== [5] 建图结果 ====="
echo "--- terrain 日志尾部 ---"
grep -iE "terrain|map|point|height|hz|ready|subscri" /tmp/terrain.log 2>/dev/null | tail -8
echo "--- mapping 日志尾部 ---"
tail -5 /tmp/mapping.log 2>/dev/null

echo "===== 收尾 ====="
kill $MPID $TPID 2>/dev/null
wait 2>/dev/null
echo "完成"
