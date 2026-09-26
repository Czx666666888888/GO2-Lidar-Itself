#!/bin/bash
# 用 rosbag 建图(通用), 回放指定 bag 看建图效果
# 用法: bash build_map_from_bag.sh <bag路径> [输出目录]
BAG=${1:?用法: bash build_map_from_bag.sh <bag路径> [输出目录]}
OUTDIR=${2:-/home/lch/dog/tmp/mapping_bag}

source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
mkdir -p "$OUTDIR"
rm -f "$OUTDIR"/*.npy "$OUTDIR"/*.npz 2>/dev/null

echo "===== bag: $BAG ====="
echo "===== 输出: $OUTDIR ====="

ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false > /tmp/mapping.log 2>&1 &
MPID=$!
ros2 launch terrain_analysis terrain_analysis.launch > /tmp/terrain.log 2>&1 &
TPID=$!
sleep 10

python3 /home/lch/dog/scripts/go2_global_map.py 90 "$OUTDIR" > "$OUTDIR/global.log" 2>&1 &
GPID=$!

echo "===== 回放 bag ====="
timeout 90 ros2 bag play "$BAG" > /dev/null 2>&1

wait $GPID 2>/dev/null
kill $MPID $TPID 2>/dev/null
wait 2>/dev/null

echo "===== 建图结果 ====="
cat "$OUTDIR/global.log"
echo "===== 完成 ====="
