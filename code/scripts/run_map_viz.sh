#!/bin/bash
# ============================================================
# 回放 rosbag -> 运行 transform_sensors + Point-LIO + terrain_analysis
# -> 累积三套点云地图 -> 渲染俯视图+斜视图
# 用法: bash scripts/run_map_viz.sh <输入bag目录> <输出名> <bag时长秒>
# 例:   bash scripts/run_map_viz.sh \
#         rosbags/go2_controlled_20260824/controlled_square_2mx2m_ccw_20260824_01 \
#         square_2x2m 243
# ============================================================
set -e
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
cd /home/lch/dog

INPUT="$1"; OUTNAME="$2"; BAGDUR="${3:-60}"
DATA=/home/lch/dog/tmp/mapdata_$OUTNAME
OUT=/home/lch/dog/outputs/pointcloud_maps/$OUTNAME
LOG=/home/lch/dog/tmp/run_mapviz_$OUTNAME.log
rm -rf "$DATA"; mkdir -p "$DATA" "$OUT"
exec > "$LOG" 2>&1
echo "===== mapviz $OUTNAME 开始 $(date) ====="
echo "bag=$INPUT dur=${BAGDUR}s"

# 1) Point-LIO (含 transform_sensors)
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false &
LPID=$!
sleep 8

# 2) terrain_analysis
ros2 launch terrain_analysis terrain_analysis.launch &
TPID=$!
sleep 6

# 3) 采集器 (bag 时长 + 25s 缓冲)
COLL_DUR=$(python3 -c "print($BAGDUR + 25)")
python3 /home/lch/dog/scripts/collect_pointcloud_maps.py "$DATA" "$COLL_DUR" &
CPID=$!
sleep 2

# 4) 回放 (实时速率)
echo ">>> 开始回放"
ros2 bag play "$INPUT" >/dev/null 2>&1
echo ">>> 回放结束"

# 5) 等采集器写完
wait $CPID
kill $LPID $TPID 2>/dev/null || true
sleep 2
pkill -f pointlio_mapping 2>/dev/null || true
pkill -f terrainAnalysis 2>/dev/null || true
pkill -f transform_everything 2>/dev/null || true

# 6) 渲染
python3 /home/lch/dog/scripts/render_pointcloud_maps.py "$DATA" "$OUT"
echo "===== mapviz $OUTNAME 完成 $(date) ====="
ls -la "$OUT"
