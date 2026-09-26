#!/bin/bash
# 用法: bash scripts/replay_record.sh <输入bag> <输出名> [duration秒]
# 回放链 → 录制 /terrain_map /state_estimation 到 tmp/analysis_<输出名>/
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
cd /home/lch/dog
INPUT="$1"; OUTNAME="$2"; DUR="${3:-0}"
OUTDIR=/home/lch/dog/tmp/analysis_$OUTNAME
rm -rf "$OUTDIR"; mkdir -p "$OUTDIR"
LOG=/home/lch/dog/tmp/replay_$OUTNAME.log
exec > "$LOG" 2>&1

ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false &
sleep 7
ros2 launch terrain_analysis terrain_analysis.launch &
sleep 6

ros2 bag record -o "$OUTDIR" /terrain_map /state_estimation >/dev/null 2>&1 &
RECPID=$!
sleep 3

if [ "$DUR" -gt 0 ]; then
  timeout "$DUR" ros2 bag play "$INPUT" >/dev/null 2>&1
else
  ros2 bag play "$INPUT" >/dev/null 2>&1
fi
sleep 4
kill -INT "$RECPID" 2>/dev/null
sleep 3
kill -TERM "$RECPID" 2>/dev/null
sleep 1
echo "RECORD_DONE $OUTDIR"
ls -la "$OUTDIR" 2>/dev/null | head
