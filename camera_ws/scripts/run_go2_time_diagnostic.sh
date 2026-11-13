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
if [ "${1:-}" = --pc-time-check ]; then
  shift
  exec /usr/bin/python3 -u "$go2_time_repo/camera_ws/scripts/pc_time_check.py" "$@"
fi
if [ "${1:-}" = --legacy-check ]; then
  shift
  echo '只读legacy定位检查：原生雷达 → 准备受影响包 → 伴随主机对时 → 定位/D435/MAP；不运动。'
  bash "$go2_time_repo/camera_ws/scripts/run_go2_time_diagnostic.sh" \
    --lidar-check --duration 10 --label legacy_native_preflight
  export GO2_TIME_MAPPING_MODE=legacy
  bash "$go2_time_repo/camera_ws/scripts/run_go2_time_diagnostic.sh" --prepare-only
  if [ "${GO2_CAMERA_HOST:-unitree@192.168.123.18}" = unitree@192.168.123.18 ]; then
    /usr/bin/python3 "$go2_time_repo/camera_ws/scripts/sync_go2_clock.py"
  fi
  exec bash "$go2_time_repo/camera_ws/scripts/run_go2_time_diagnostic.sh" \
    --time-mode legacy --restart-localization --require-target --label legacy_localization "$@"
fi
source /opt/ros/humble/setup.bash
source "${GO2_TIME_UNDERLAY:-$go2_time_repo/deploy_ws/install/setup.bash}"
go2_time_observe=false
for go2_time_arg in "$@"; do
  case "$go2_time_arg" in --observe-only|--lidar-check|--help|-h) go2_time_observe=true ;; esac
done
if [ "$go2_time_observe" = false ]; then
  go2_time_perception="$go2_time_repo/camera_ws/src/go2_science_perception"
  go2_time_perception_hash="$(sha256sum "$go2_time_perception/config/science_target_approach.yaml" "$go2_time_perception/go2_science_perception/science_target_approach.py" | sha256sum | cut -d' ' -f1)"
  go2_time_perception_marker="$go2_time_repo/.git/time_build_perception_config"
  if [ ! -f "$go2_time_repo/.onekey/install/go2_science_perception/share/go2_science_perception/package.xml" ] || [ ! -f "$go2_time_perception_marker" ] || [ "$(cat "$go2_time_perception_marker")" != "$go2_time_perception_hash" ]; then
    (cd "$go2_time_repo" && PYTHONNOUSERSITE=1 /usr/bin/python3 /usr/bin/colcon \
      --log-base .onekey/log build --base-paths camera_ws/src \
      --build-base .onekey/build --install-base .onekey/install --packages-select go2_science_perception)
    printf '%s\n' "$go2_time_perception_hash" > "$go2_time_perception_marker"
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
