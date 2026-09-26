#!/bin/bash
# 用标定后的参数, 回放指定数据目录建图
# 用法: bash build_map_from_data.sh <数据目录> [输出目录]
DATA_DIR=${1:?用法: bash build_map_from_data.sh <数据目录> [输出目录]}
OUTDIR=${2:-/home/lch/dog/tmp/mapping_result}

source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
mkdir -p "$OUTDIR"
rm -f "$OUTDIR"/*.npy "$OUTDIR"/*.npz 2>/dev/null

echo "===== 数据目录: $DATA_DIR ====="
echo "===== 输出目录: $OUTDIR ====="

echo "===== [1] 启动建图链(读标定 yaml) ====="
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false > /tmp/mapping.log 2>&1 &
MPID=$!
ros2 launch terrain_analysis terrain_analysis.launch > /tmp/terrain.log 2>&1 &
TPID=$!
sleep 10

echo "===== [2] 累积全局地图(115s) ====="
python3 /home/lch/dog/scripts/go2_global_map.py 115 "$OUTDIR" > "$OUTDIR/global.log" 2>&1 &
GPID=$!

echo "===== [3] 回放数据 ====="
python3 /home/lch/dog/scripts/replay_dds_data.py \
    --imu_csv "$DATA_DIR/imu.csv" --lidar_npz "$DATA_DIR/lidar.npz" \
    > "$OUTDIR/replay.log" 2>&1 &
RPID=$!

wait $RPID 2>/dev/null
wait $GPID 2>/dev/null

kill $MPID $TPID 2>/dev/null
wait 2>/dev/null

echo "===== 建图结果 ====="
cat "$OUTDIR/global.log"
echo "===== 输出文件 ====="
ls -la "$OUTDIR/"
echo "===== 完成 ====="
