#!/bin/bash
for pat in laserMapping terrainAnalysis transform_everything pointlio_mapping "ros2 bag play" "ros2 topic" "ros2 launch"; do
  pkill -9 -f "$pat" 2>/dev/null
done
sleep 2
echo "残留 ROS 进程: $(ps aux 2>/dev/null | grep -E 'laserMapping|terrainAnalysis|transform_everything|pointlio' | grep -v grep | wc -l)"
