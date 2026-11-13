#!/usr/bin/env bash
# All sensor/localization timestamps and processes belong to this host.
set -eo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
profile="${GO2_PROFILE_FILE:-$repo/config/go2_local.env}"
if [ -f "$profile" ]; then source "$profile"; fi
export GO2_RUNTIME_PROFILE=local
export GO2_CAMERA_HOST=local
export GO2_TIME_MAPPING_MODE="${GO2_TIME_MAPPING_MODE:-legacy}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
ros_setup="${GO2_ROS_SETUP:-}"
if [ -z "$ros_setup" ]; then
  for distro in humble foxy; do
    if [ -f "/opt/ros/$distro/setup.bash" ]; then ros_setup="/opt/ros/$distro/setup.bash"; break; fi
  done
fi
[ -f "$ros_setup" ] || { echo 'BLOCKED: 设置GO2_ROS_SETUP为本机ROS setup.bash'; exit 2; }
source "$ros_setup"
for setup in "${GO2_DDS_SETUP:-}" "${GO2_CAMERA_SETUP:-}" "${GO2_TIME_UNDERLAY:-$repo/deploy_ws/install/setup.bash}" "$repo/.onekey/install/setup.bash" "$repo/.time_sync/install/setup.bash"; do
  if [ -n "$setup" ] && [ -f "$setup" ]; then source "$setup"; fi
done
if [ -z "${CYCLONEDDS_URI:-}" ] && [ -n "${GO2_TIME_INTERFACE:-}" ]; then
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>$GO2_TIME_INTERFACE</NetworkInterfaceAddress></General></Domain></CycloneDDS>"
fi
# No PC peer, address, NTP or SSH. DDS may select a local interface automatically.
missing=()
for pkg in point_lio_unilidar transform_sensors go2_keyboard_teleop go2_science_perception realsense2_camera "$RMW_IMPLEMENTATION"; do
  ros2 pkg prefix "$pkg" >/dev/null 2>&1 || missing+=("$pkg")
done
if [ "${#missing[@]}" -gt 0 ]; then echo "BLOCKED: 本机缺少 ${missing[*]}；未启动传感器/定位/运动。"; exit 2; fi
if [ "${1:-}" = --execute ]; then
  echo 'BLOCKED: NX本机完整S1/控制入口尚未验收；此入口仅进行只读定位。'; exit 2
fi
exec /usr/bin/python3 -u "$repo/camera_ws/scripts/go2_time_diagnostic.py" \
  --camera-host local --time-mode "$GO2_TIME_MAPPING_MODE" --restart-localization --require-target "$@"
