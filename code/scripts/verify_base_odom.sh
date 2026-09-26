#!/bin/bash
# 验证 base_odom_node 转换: 只起 Point-LIO + base_odom + 采集, 回放一小段
# 用法: bash scripts/verify_base_odom.sh <bag目录>
BAG=${1:?用法: bash verify_base_odom.sh <bag目录>}
OUT=/home/lch/dog/tmp/verify_baseodom

source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
source /home/lch/dog/go2_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=88
unset CYCLONEDDS_URI    # 关键: 用 loopback, 不用 enp8s0

mkdir -p "$OUT"

ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false > "$OUT/mapping.log" 2>&1 &
MPID=$!
ros2 run go2_keyboard_teleop base_odom_node > "$OUT/base.log" 2>&1 &
BPID=$!
echo "等初始化 12s..."
sleep 12

python3 /home/lch/dog/scripts/verify_base_odom.py 45 > "$OUT/verify.log" 2>&1 &
VPID=$!
sleep 3

timeout 50 ros2 bag play "$BAG" --topics /utlidar/cloud /utlidar/imu > "$OUT/play.log" 2>&1
wait $VPID 2>/dev/null

kill $MPID $BPID 2>/dev/null
sleep 1
echo "===== 验证结果 ====="
cat "$OUT/verify.log" 2>/dev/null
