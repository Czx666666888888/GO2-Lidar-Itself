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
source "$go2_time_repo/deploy_ws/install/setup.bash"
ssh -i /home/czx666/.ssh/id_ed25519_go2 -o BatchMode=yes -o ConnectTimeout=5 \
  unitree@192.168.123.18 true
# Leave existing SLAM/transform processes and their map state untouched.
if ! pgrep -f '^.*/(pointlio_mapping|transform_everything)( |$)' >/dev/null; then
  for go2_time_package in point_lio_unilidar transform_sensors; do
    if [ "$go2_time_package" = point_lio_unilidar ]; then
      go2_time_path="$go2_time_repo/code/autonomy_stack_go2_src/slam/point_lio_unilidar"
      go2_time_hash="$(sha256sum "$go2_time_path/src/laserMapping.cpp" "$go2_time_path/launch/mapping_utlidar.launch" | sha256sum | cut -d' ' -f1)"
    else
      go2_time_path="$go2_time_repo/code/autonomy_stack_go2_src/utilities/transform_sensors"
      go2_time_hash="$(sha256sum "$go2_time_path/transform_sensors/transform_everything.py" | sha256sum | cut -d' ' -f1)"
    fi
    go2_time_marker="$go2_time_repo/.git/time_build_$go2_time_package"
    if [ ! -f "$go2_time_marker" ] || [ "$(cat "$go2_time_marker")" != "$go2_time_hash" ] || [ ! -f "$go2_time_repo/.time_sync/install/$go2_time_package/share/$go2_time_package/package.xml" ]; then
      (cd "$go2_time_repo" && PYTHONNOUSERSITE=1 CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 \
        /usr/bin/python3 -X faulthandler /usr/bin/colcon --log-base .time_sync/log build \
        --base-paths "$go2_time_path" --build-base .time_sync/build \
        --install-base .time_sync/install --packages-select "$go2_time_package" --executor sequential)
      printf '%s\n' "$go2_time_hash" > "$go2_time_marker"
    fi
  done
fi
if [ -f "$go2_time_repo/.time_sync/install/setup.bash" ]; then
  source "$go2_time_repo/.time_sync/install/setup.bash"
fi
source "$go2_time_repo/camera_ws/scripts/science_approach_pc_env.bash"
exec /usr/bin/python3 -u "$go2_time_repo/camera_ws/scripts/go2_time_diagnostic.py" "$@"
