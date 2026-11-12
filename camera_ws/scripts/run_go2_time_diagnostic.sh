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
# Pure PC-side DDS subscription: no SSH, rebuild, configuration or SLAM launch.
if [ -f "$go2_time_repo/.time_sync/install/setup.bash" ]; then
  source "$go2_time_repo/.time_sync/install/setup.bash"
fi
source "$go2_time_repo/camera_ws/scripts/science_approach_pc_env.bash"
exec /usr/bin/python3 -u "$go2_time_repo/camera_ws/scripts/go2_time_diagnostic.py" "$@"
