#!/usr/bin/env bash
# Calling with --execute is the operator's explicit start action; no further prompts.
set -o pipefail
approach_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$approach_repo_dir/deploy_ws/install/setup.bash"
approach_overlay="${SCIENCE_APPROACH_OVERLAY:-$approach_repo_dir/camera_ws/install/setup.bash}"
if [ ! -f "$approach_overlay" ]; then
  echo "Missing camera overlay: $approach_overlay" >&2
  exit 1
fi
source "$approach_overlay"
source "$approach_repo_dir/camera_ws/scripts/science_approach_pc_env.bash"
exec /usr/bin/python3 -u "$approach_repo_dir/camera_ws/scripts/run_map_anchor_experiment.py" "$@"
