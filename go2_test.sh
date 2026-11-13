#!/usr/bin/env bash
# Motion: --execute --mode dock. Read-only reuse/timing: --startup-check.
# Full 40s clock sampling is explicit: append --clock-diagnostic to --execute.
set -euo pipefail
go2_test_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [ "$(uname -m)" = aarch64 ]; then
  export GO2_RUNTIME_PROFILE="${GO2_RUNTIME_PROFILE:-local}"
fi
if [ "${GO2_RUNTIME_PROFILE:-}" != local ]; then
  export GO2_TIME_MAPPING_MODE="${GO2_TIME_MAPPING_MODE:-imu}"
  if [ "${1:-}" = --execute ] && [ "${GO2_PC_TIME_HELD:-}" != 1 ]; then
    exec /usr/bin/python3 -u "$go2_test_root/camera_ws/scripts/pc_time_check.py" "$@"
  fi
fi
if [ "${GO2_RUNTIME_PROFILE:-}" = local ] && [ "$#" -eq 0 ]; then
  exec bash "$go2_test_root/camera_ws/scripts/run_go2_local.sh"
fi
if [ "${1:-}" = --local-check ]; then
  shift
  exec bash "$go2_test_root/camera_ws/scripts/run_go2_local.sh" "$@"
fi
if [ "${GO2_RUNTIME_PROFILE:-}" = local ] && [ "${1:-}" = --execute ]; then
  exec bash "$go2_test_root/camera_ws/scripts/run_go2_local.sh" "$@"
fi
if [ "${1:-}" = --time-diagnostic ]; then
  shift
  exec bash "$go2_test_root/camera_ws/scripts/run_go2_time_diagnostic.sh" "$@"
fi
if [ "${1:-}" = --execute ] && [ "$(git -C "$go2_test_root" branch --show-current)" != feature/map-approach-s1-grasp-integration ]; then
  go2_test_integration=/tmp/go2_onekey_s1
  if [ ! -f "$go2_test_integration/go2_test.sh" ] || [ "$go2_test_root" = "$go2_test_integration" ]; then
    echo 'S1集成入口不可用；先运行 bash camera_ws/scripts/run_go2_time_diagnostic.sh 恢复并诊断。' >&2
    exit 1
  fi
  exec bash "$go2_test_integration/go2_test.sh" "$@"
fi
exec /usr/bin/python3 "$go2_test_root/scripts/go2_full_test.py" "$@"
