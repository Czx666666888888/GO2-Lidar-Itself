#!/usr/bin/env bash
# Run ON the GO2 companion host; source-built official wrapper 4.54.1.
set -eo pipefail
source /opt/ros/foxy/setup.bash
source /home/unitree/go2_d435_foxy_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Avoid discovery of incompatible native GO2 participants; camera talks to PC only.
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>eth0</NetworkInterfaceAddress><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer Address="192.168.123.99"/></Peers><ParticipantIndex>auto</ParticipantIndex></Discovery></Domain></CycloneDDS>'
# Optional explicit binding when the S1 wrist camera is also connected.
camera_serial_args=()
if [ -n "${GO2_D435_SERIAL:-}" ]; then
  camera_serial_args=(-p "serial_no:='$GO2_D435_SERIAL'")
fi
exec ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -r __ns:=/camera -r __node:=camera \
  --params-file /home/unitree/go2_d435_foxy_ws/realsense_go2_foxy.yaml \
  "${camera_serial_args[@]}" \
  -r /camera/color/image_raw:=/camera/camera/color/image_raw \
  -r /camera/color/camera_info:=/camera/camera/color/camera_info \
  -r /camera/aligned_depth_to_color/image_raw:=/camera/camera/aligned_depth_to_color/image_raw \
  -r /camera/aligned_depth_to_color/camera_info:=/camera/camera/aligned_depth_to_color/camera_info \
  -r /camera/depth/camera_info:=/camera/camera/depth/camera_info
