#!/bin/bash
# WP2: 全局地图构建 + home_anchor (正方形包回放)
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
cd /home/lch/dog
exec > /home/lch/dog/tmp/wp2.log 2>&1
INPUT="$1"; DUR="${2:-130}"
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false &
sleep 7
ros2 launch terrain_analysis terrain_analysis.launch &
sleep 6
python3 /home/lch/dog/scripts/go2_global_map.py "$DUR" &
GMPID=$!
sleep 3
timeout "$DUR" ros2 bag play "$INPUT" >/dev/null 2>&1 &
wait $GMPID 2>/dev/null
echo "WP2_DONE"
