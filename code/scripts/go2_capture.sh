#!/bin/bash
# go2_capture.sh: 采集一次传感器数据(自动时间戳命名目录, 支持多次采集不覆盖)
#
# 用法:
#   bash go2_capture.sh                    # 手动采集, Ctrl-C 停止保存
#   bash go2_capture.sh 60                 # 定时采集 60 秒自动停止
#   bash go2_capture.sh 30 static          # 采集 30 秒, 标签 static(如静止标定)
#   bash go2_capture.sh 90 motion          # 采集 90 秒, 标签 motion(如运动建图)
#
# 每次采集自动保存到: /home/lch/dog/go2_data/<标签>_<时间戳>/
#   含 imu.csv / odom.csv / lidar.npz, 互不覆盖。

DUR=${1:-0}          # 采集时长(秒), 0=手动 Ctrl-C
TAG=${2:-session}    # 采集标签

TS=$(date +%Y%m%d_%H%M%S)
OUTDIR=/home/lch/dog/go2_data/${TAG}_${TS}
mkdir -p "$OUTDIR"

source ~/dog/go2_ws/setup_dds.sh

echo "======================================"
echo "采集会话: $TAG"
echo "输出目录: $OUTDIR"
echo "时长: $([ "$DUR" -gt 0 ] && echo "${DUR}s 定时" || echo "手动(Ctrl-C 停止)")"
echo "======================================"

if [ "$DUR" -gt 0 ]; then
    ros2 run go2_keyboard_teleop go2_sensor_recorder_dds \
        --ros-args -p outdir:="$OUTDIR" -p duration:="$DUR.0"
else
    ros2 run go2_keyboard_teleop go2_sensor_recorder_dds \
        --ros-args -p outdir:="$OUTDIR"
fi

echo "======================================"
echo "采集完成, 数据在: $OUTDIR"
ls -la "$OUTDIR"
echo "======================================"
