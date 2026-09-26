#!/bin/bash
# go2_record_bag.sh: 用 ros2 bag record 录制传感器数据(和团队 rosbag 数据结构一致)
#
# 录制话题(与 go2_controlled_20260824 的三个 rosbag 完全一致):
#   /utlidar/imu         (sensor_msgs/Imu)
#   /utlidar/cloud       (sensor_msgs/PointCloud2, 含 x/y/z/intensity/ring/time)
#   /utlidar/robot_odom  (nav_msgs/Odometry, 含位置+朝向四元数)
#   /lf/sportmodestate   (unitree_go/SportModeState)
#
# 用法:
#   bash go2_record_bag.sh 60              # 录 60 秒, 自动停止
#   bash go2_record_bag.sh 120 obstacle    # 录 120 秒, 标签 obstacle
#   bash go2_record_bag.sh                 # 手动 Ctrl-C 停止
#
# 输出: /home/lch/dog/rosbags/go2_record_<标签>_<时间戳>/ (标准 rosbag2)

DUR=${1:-0}
TAG=${2:-session}

TS=$(date +%Y%m%d_%H%M%S)
OUT=/home/lch/dog/rosbags/go2_record_${TAG}_${TS}

source ~/dog/go2_ws/setup_dds.sh

TOPICS="/utlidar/imu /utlidar/cloud /utlidar/robot_odom /lf/sportmodestate"

echo "======================================"
echo "录制 rosbag: $TAG"
echo "输出: $OUT"
echo "话题: $TOPICS"
echo "时长: $([ "$DUR" -gt 0 ] && echo "${DUR}s 定时" || echo "手动(Ctrl-C 停止)")"
echo "======================================"

if [ "$DUR" -gt 0 ]; then
    timeout "$DUR" ros2 bag record $TOPICS -o "$OUT"
else
    ros2 bag record $TOPICS -o "$OUT"
fi

echo "======================================"
echo "录制完成: $OUT"
ls -la "$OUT"
echo "======================================"
