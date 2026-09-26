#!/bin/bash
# ============================================================
# 离线建图链 + 渲染: 回放 rosbag → Point-LIO → terrain → 栅格地图/高度图/轨迹图
# 用法: bash scripts/offline_map_render.sh <bag目录> <输出目录>
# 例:   bash scripts/offline_map_render.sh rosbags/go2_controlled_20260827/obstacle_full_scene_handheld_20260827_02 /home/lch/dog/tmp/map_obstacle02
# 注意: 不能用 set -u —— conda 激活脚本引用未绑定变量 CONDA_BUILD 会直接退出
# ============================================================
BAG=${1:?用法: bash offline_map_render.sh <bag目录> <输出目录>}
OUT=${2:-/home/lch/dog/tmp/map_offline}

source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1

# 取 bag 时长(用于决定采集时长)
BAG_DUR=$(ros2 bag info "$BAG" 2>/dev/null | awk '/Duration:/{printf "%.0f", $2}')
[ -z "$BAG_DUR" ] && BAG_DUR=232
COLLECT_DUR=$(( BAG_DUR + 30 ))   # 回放完再采 30s 收尾

mkdir -p "$OUT"
rm -f "$OUT"/*.npz "$OUT"/*.npy "$OUT"/*.png 2>/dev/null

echo "=============================================="
echo "离线建图链回放: $BAG"
echo "bag 时长 ${BAG_DUR}s | 采集时长 ${COLLECT_DUR}s | 输出 $OUT"
echo "=============================================="

# 1. 启动建图链 (transform_sensors + Point-LIO + terrain_analysis)
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false > "$OUT/mapping.log" 2>&1 &
MPID=$!
ros2 launch terrain_analysis terrain_analysis.launch > "$OUT/terrain.log" 2>&1 &
TPID=$!
echo "[1] 建图链已启动, 等待初始化 12s..."
sleep 12

# 2. 采集器(订阅 /terrain_map + /state_estimation, 存 global_map.npz + trajectory.npy)
python3 /home/lch/dog/scripts/go2_global_map.py "$COLLECT_DUR" "$OUT" > "$OUT/global.log" 2>&1 &
GPID=$!
sleep 3

# 3. 回放整个 bag (实时速率)
echo "[2] 回放 bag (${BAG_DUR}s)..."
ros2 bag play "$BAG" > "$OUT/play.log" 2>&1
echo "[3] bag 回放完成"

# 4. 等采集器收尾
echo "[4] 等待采集器收尾($((COLLECT_DUR - BAG_DUR - 3))s)..."
wait $GPID 2>/dev/null

# 5. 关建图链
kill $MPID $TPID 2>/dev/null
pkill -f pointlio_mapping 2>/dev/null
pkill -f transform_everything 2>/dev/null
pkill -f terrainAnalysis 2>/dev/null
sleep 2

# 6. 渲染图片
echo "[5] 渲染图片..."
python3 /home/lch/dog/scripts/wp2_grid.py "$OUT" > "$OUT/render_grid.log" 2>&1
python3 /home/lch/dog/scripts/render_height_map.py "$OUT" > "$OUT/render_height.log" 2>&1

echo "=============================================="
echo "建图结果:"
cat "$OUT/global.log" 2>/dev/null | tail -15
echo "----- 栅格 -----"
cat "$OUT/render_grid.log" 2>/dev/null | tail -8
echo "----- 高度图 -----"
cat "$OUT/render_height.log" 2>/dev/null
echo ""
echo "输出图片:"
ls -la "$OUT"/*.png 2>/dev/null
echo "=============================================="
