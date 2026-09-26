#!/bin/bash
# 正方形路径完整成果跑一遍: 建图→全局地图→三态栅格→前沿→返航→总览
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
cd /home/lch/dog
exec > /home/lch/dog/tmp/run_all.log 2>&1

BAG=/home/lch/dog/rosbags/go2_controlled_20260824/controlled_square_2mx2m_ccw_20260824_01

echo "===== [1/4] 启动建图链 (Point-LIO + terrain) ====="
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false &
sleep 7
ros2 launch terrain_analysis terrain_analysis.launch &
sleep 6

echo "===== [2/4] 全局地图构建 + 轨迹记录 (130s) ====="
python3 /home/lch/dog/scripts/go2_global_map.py 130 &
GMPID=$!
sleep 3

echo "===== [3/4] 回放正方形包 (前130s有效段) ====="
timeout 130 ros2 bag play "$BAG" >/dev/null 2>&1
wait $GMPID 2>/dev/null

echo "===== [4/4] 离线分析链 ====="
python3 /home/lch/dog/scripts/wp2_grid.py 2>&1 | grep -E "三态|occupancy|已保存" || true
python3 /home/lch/dog/scripts/wp5_frontier_v2.py 2>&1 | grep -E "合并后|可达|最优|已保存" || true
python3 /home/lch/dog/scripts/wp6_return.py 2>&1 | grep -E "关键帧|HOME|返航路径|已保存" || true
python3 /home/lch/dog/scripts/wp2_overview.py 2>&1 | grep -E "已保存" || true

echo "===== 完成 ====="
ls -la /home/lch/dog/tmp/wp2/*.png 2>/dev/null
echo "RUN_ALL_DONE"
