#!/usr/bin/env bash
# Operator's separate terminal: source the same read-only DDS environment.
set -o pipefail
approach_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$approach_repo_dir/deploy_ws/install/setup.bash"
source "$approach_repo_dir/camera_ws/scripts/science_approach_pc_env.bash"
exec /usr/bin/python3 -u "$approach_repo_dir/camera_ws/scripts/watch_approach_status.py"
