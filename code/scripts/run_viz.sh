#!/bin/bash
# 回放正方形包(有效前130s) + 累积渲染 terrain 效果图
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
cd /home/lch/dog
LOG=/home/lch/dog/tmp/viz.log
exec > "$LOG" 2>&1
BAG=/home/lch/dog/rosbags/go2_controlled_20260824/controlled_stationary_20260824_01
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false &
sleep 7
ros2 launch terrain_analysis terrain_analysis.launch &
sleep 6
# 渲染节点(累积 140s)
python3 /home/lch/dog/scripts/render_terrain.py &
VPID=$!
sleep 3
# 播放静止包
ros2 bag play "$BAG"  >/dev/null 2>&1 &
BPID=$!
wait $BPID 2>/dev/null
wait $VPID 2>/dev/null
echo "=== 完成 ==="
ls -la /home/lch/dog/tmp/terrain_effect.png 2>/dev/null
