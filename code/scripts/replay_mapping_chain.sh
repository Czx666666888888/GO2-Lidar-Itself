#!/bin/bash
# 用 rosbag 的原始传感器数据跑一遍建图链(Point-LIO + terrain), 出栅格图+高度图
# 用法: bash scripts/replay_mapping_chain.sh <bag目录> [输出目录]
# 注意: 只回放 /utlidar/cloud + /utlidar/imu, 避免和 bag 里已录的处理话题冲突
BAG=${1:?用法: bash replay_mapping_chain.sh <bag目录> [输出目录]}
OUT=${2:-/home/lch/dog/tmp/map_fullchain}

source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1

BAG_DUR=$(ros2 bag info "$BAG" 2>/dev/null | awk '/Duration:/{printf "%.0f", $2}')
[ -z "$BAG_DUR" ] && BAG_DUR=140
COLLECT_DUR=$(( BAG_DUR + 30 ))

mkdir -p "$OUT"
rm -f "$OUT"/*.npz "$OUT"/*.npy "$OUT"/*.png 2>/dev/null

echo "=============================================="
echo "离线建图链回放: $BAG"
echo "bag ${BAG_DUR}s | 采集 ${COLLECT_DUR}s | 输出 $OUT"
echo "=============================================="

# 1. 建图链 (transform_sensors + Point-LIO + terrain_analysis)
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false > "$OUT/mapping.log" 2>&1 &
MPID=$!
ros2 launch terrain_analysis terrain_analysis.launch > "$OUT/terrain.log" 2>&1 &
TPID=$!
echo "[1] 建图链已启动, 等初始化 12s..."
sleep 12

# 2. 采集器(订阅 /terrain_map + /state_estimation)
python3 /home/lch/dog/scripts/go2_global_map.py "$COLLECT_DUR" "$OUT" > "$OUT/global.log" 2>&1 &
GPID=$!
sleep 3

# 3. 只回放原始传感器话题
echo "[2] 回放原始传感器(/utlidar/cloud + /utlidar/imu)..."
ros2 bag play "$BAG" --topics /utlidar/cloud /utlidar/imu > "$OUT/play.log" 2>&1
echo "[3] 回放完成"

# 4. 等采集器收尾
wait $GPID 2>/dev/null

# 5. 关建图链
kill $MPID $TPID 2>/dev/null
pkill -f pointlio_mapping 2>/dev/null
pkill -f transform_everything 2>/dev/null
pkill -f terrainAnalysis 2>/dev/null
sleep 2

# 6. 渲染
echo "[4] 渲染图片..."
python3 /home/lch/dog/scripts/wp2_grid.py "$OUT" > "$OUT/render_grid.log" 2>&1
python3 /home/lch/dog/scripts/render_height_map.py "$OUT" > "$OUT/render_height.log" 2>&1

echo "=============================================="
echo "建图结果:"
tail -12 "$OUT/global.log" 2>/dev/null
echo "----- 栅格 -----"
tail -6 "$OUT/render_grid.log" 2>/dev/null
echo "----- 高度图 -----"
cat "$OUT/render_height.log" 2>/dev/null
echo "----- 输出图片 -----"
ls -la "$OUT"/*.png 2>/dev/null
echo "=============================================="
