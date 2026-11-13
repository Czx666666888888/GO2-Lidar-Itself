#!/usr/bin/env bash
# Read-only hardware diagnostic; no motion publisher and no clock changes.
set -eo pipefail
go2_time_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
# This same entry can be placed in the main checkout, without switching its
# branch or editing its dirty source. The implementation remains on S1 branch.
if [ ! -f "$go2_time_repo/camera_ws/scripts/go2_time_diagnostic.py" ]; then
  go2_time_integration=/tmp/go2_onekey_s1
  if [ ! -d "$go2_time_integration/.git" ]; then
    git clone --branch feature/map-approach-s1-grasp-integration --single-branch \
      https://github.com/Czx666666888888/GO2-Lidar-Itself.git "$go2_time_integration"
  fi
  if [ ! -e "$go2_time_integration/deploy_ws" ]; then
    ln -s "$go2_time_repo/deploy_ws" "$go2_time_integration/deploy_ws"
  fi
  mkdir -p "$go2_time_repo/camera_ws/records"
  if [ ! -e "$go2_time_integration/camera_ws/records" ]; then
    ln -s "$go2_time_repo/camera_ws/records" "$go2_time_integration/camera_ws/records"
  fi
  exec bash "$go2_time_integration/camera_ws/scripts/run_go2_time_diagnostic.sh" "$@"
fi
source /opt/ros/humble/setup.bash
source "${GO2_TIME_UNDERLAY:-$go2_time_repo/deploy_ws/install/setup.bash}"
go2_time_observe=false
for go2_time_arg in "$@"; do
  case "$go2_time_arg" in --observe-only|--help|-h) go2_time_observe=true ;; esac
done
if [ "$go2_time_observe" = false ]; then
  if [ ! -f "$go2_time_repo/.onekey/install/go2_science_perception/share/go2_science_perception/package.xml" ]; then
    (cd "$go2_time_repo" && PYTHONNOUSERSITE=1 /usr/bin/python3 /usr/bin/colcon \
      --log-base .onekey/log build --base-paths camera_ws/src \
      --build-base .onekey/build --install-base .onekey/install --packages-select go2_science_perception)
  fi
  go2_time_source="$go2_time_repo/code/autonomy_stack_go2_src/utilities/transform_sensors"
  go2_time_hash="$(sha256sum "$go2_time_source/transform_sensors/transform_everything.py" "$go2_time_source/transform_sensors/imu_time_mapping.py" "$go2_time_source/package.xml" | sha256sum | cut -d' ' -f1)"
  go2_time_marker="$go2_time_repo/.git/time_build_transform_sensors"
  if [ ! -f "$go2_time_marker" ] || [ "$(cat "$go2_time_marker")" != "$go2_time_hash" ] || [ ! -f "$go2_time_repo/.time_sync/install/transform_sensors/share/transform_sensors/package.xml" ]; then
    (cd "$go2_time_repo" && PYTHONNOUSERSITE=1 /usr/bin/python3 -X faulthandler /usr/bin/colcon \
      --log-base .time_sync/log build --base-paths "$go2_time_source" \
      --build-base .time_sync/build --install-base .time_sync/install --packages-select transform_sensors)
    printf '%s\n' "$go2_time_hash" > "$go2_time_marker"
  fi
fi
if [ -f "$go2_time_repo/.onekey/install/setup.bash" ]; then
  source "$go2_time_repo/.onekey/install/setup.bash"
fi
if [ -f "$go2_time_repo/.time_sync/install/setup.bash" ]; then
  source "$go2_time_repo/.time_sync/install/setup.bash"
fi
if [ -z "${CYCLONEDDS_URI:-}" ]; then
  go2_time_iface="${GO2_TIME_INTERFACE:-enp12s0}"
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>$go2_time_iface</NetworkInterfaceAddress></General><Discovery><Peers><Peer Address=\"192.168.123.18\"/></Peers><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>60</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"
fi
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
exec /usr/bin/python3 -u "$go2_time_repo/camera_ws/scripts/go2_time_diagnostic.py" "$@"
