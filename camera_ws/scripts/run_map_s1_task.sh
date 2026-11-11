#!/usr/bin/env bash
set -eo pipefail
s1_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$s1_repo/deploy_ws/install/setup.bash"
source "${SCIENCE_APPROACH_OVERLAY:-/tmp/science_s1_install/setup.bash}"
source "$s1_repo/camera_ws/scripts/science_approach_pc_env.bash"
exec /usr/bin/python3 -u "$s1_repo/camera_ws/scripts/run_map_s1_task.py" "$@"
