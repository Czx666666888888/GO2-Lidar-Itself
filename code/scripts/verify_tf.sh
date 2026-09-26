#!/bin/bash
# 验证 robot_state_publisher 是否发布 TF (前台内嵌启动)
# set -u  (robostack 激活脚本含未绑定变量 CONDA_BUILD, 不能用 set -u)
export HOME=/home/lch/dog/tools/home
export MAMBA_ROOT_PREFIX=/home/lch/dog/tools/mamba
export PATH=/home/lch/dog/tools/bin:$PATH
eval "$(micromamba shell hook --shell bash)"
micromamba activate /home/lch/dog/ros2/humble
source /home/lch/dog/go2_ws/install/setup.bash
cd /home/lch/dog/go2_ws

LOG=/home/lch/dog/tmp/verify_tf.log
mkdir -p /home/lch/dog/tmp

ROBOT_IP=192.168.123.161 ros2 launch go2_robot_sdk robot.launch.py \
    rviz2:=false slam:=false nav2:=false foxglove:=false joystick:=false teleop:=false \
    > "$LOG" 2>&1 &
LAUNCH_PID=$!

sleep 10

echo "===== 1. RSP 加载的 segment 数 ====="
grep -c "got segment" "$LOG" 2>/dev/null || echo 0

echo "===== 2. /tf_static 变换数 ====="
timeout 6 ros2 topic echo /tf_static --once 2>/dev/null | grep -c "child_frame_id" || echo 0

echo "===== 3. /tf_static 帧示例 ====="
timeout 6 ros2 topic echo /tf_static --once 2>/dev/null | grep "child_frame_id" | head -6

echo "===== 4. tf2_echo base_footprint -> base_link ====="
timeout 6 ros2 run tf2_ros tf2_echo base_footprint base_link 2>&1 | grep -E "Translation|At time" | head -2

kill "$LAUNCH_PID" 2>/dev/null
sleep 1
echo "===== 完成 ====="
