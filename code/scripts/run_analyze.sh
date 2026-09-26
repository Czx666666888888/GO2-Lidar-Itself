#!/bin/bash
# 用法: bash run_analyze.sh <输入bag> <输出名> [duration秒]
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
cd /home/lch/dog
INPUT="$1"; OUTNAME="$2"; DUR="${3:-60}"
exec > /home/lch/dog/tmp/analyze_$OUTNAME.log 2>&1
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false &
sleep 7
ros2 launch terrain_analysis terrain_analysis.launch &
sleep 6
python3 /home/lch/dog/scripts/analyze_live.py "$DUR" &
APID=$!
sleep 3
timeout "$DUR" ros2 bag play "$INPUT" >/dev/null 2>&1 &
BPID=$!
wait $BPID 2>/dev/null
wait $APID 2>/dev/null
echo "ANALYZE_DONE"
