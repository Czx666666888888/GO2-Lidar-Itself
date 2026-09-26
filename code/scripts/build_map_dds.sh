#!/bin/bash
# 完整建图: 启动建图链 + 回放完整采集数据 + 累积全局地图
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1

OUTDIR=/home/lch/dog/tmp/mapping_dds
mkdir -p "$OUTDIR"
rm -f "$OUTDIR"/*.npz "$OUTDIR"/*.npy 2>/dev/null

echo "===== [1] 启动建图链 ====="
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false > /tmp/mapping.log 2>&1 &
MPID=$!
ros2 launch terrain_analysis terrain_analysis.launch > /tmp/terrain.log 2>&1 &
TPID=$!
sleep 10

echo "===== [2] 启动全局地图累积(120s) ====="
python3 /home/lch/dog/scripts/go2_global_map.py 120 "$OUTDIR" > "$OUTDIR/global.log" 2>&1 &
GPID=$!

echo "===== [3] 回放完整采集数据 ====="
python3 /home/lch/dog/scripts/replay_dds_data.py > "$OUTDIR/replay.log" 2>&1 &
RPID=$!

wait $RPID
echo "===== 回放完成, 等累积收尾 ====="
wait $GPID

kill $MPID $TPID 2>/dev/null
wait 2>/dev/null

echo "===== 建图结果 ====="
cat "$OUTDIR/global.log"
echo "===== 输出文件 ====="
ls -la "$OUTDIR/"
echo "===== 完成 ====="
