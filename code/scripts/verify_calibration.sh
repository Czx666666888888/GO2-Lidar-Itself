#!/bin/bash
# 验证 IMU 标定效果: 用标定后的参数回放直线 bag, 看里程计是否不再发散
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1

BAG=/home/lch/dog/rosbags/go2_controlled_20260824/controlled_straight_2m_roundtrip_20260824_01
OUTDIR=/home/lch/dog/tmp/verify_calib
mkdir -p "$OUTDIR"
rm -f "$OUTDIR"/*.npy "$OUTDIR"/*.npz 2>/dev/null

echo "===== [1] 启动建图链 ====="
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false > /tmp/mapping.log 2>&1 &
MPID=$!
ros2 launch terrain_analysis terrain_analysis.launch > /tmp/terrain.log 2>&1 &
TPID=$!
sleep 10

echo "===== [2] 累积轨迹 + 地图 (130s) ====="
python3 /home/lch/dog/scripts/go2_global_map.py 130 "$OUTDIR" > "$OUTDIR/global.log" 2>&1 &
GPID=$!

echo "===== [3] 回放直线 bag ====="
timeout 130 ros2 bag play "$BAG" > /dev/null 2>&1

wait $GPID 2>/dev/null
kill $MPID $TPID 2>/dev/null
wait 2>/dev/null

echo "===== 标定后建图结果 ====="
cat "$OUTDIR/global.log"
echo "===== 完成 ====="
