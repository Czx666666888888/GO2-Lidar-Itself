#!/usr/bin/env bash
# Single PC entry; default only prints the plan.
set -euo pipefail
go2_test_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Explicit vision-only mode bypasses full/dock/S1 entry paths, including preview.
go2_test_previous=''
for go2_test_argument in "$@"; do
  if [ "$go2_test_argument" = --mode=approach ] || { [ "$go2_test_previous" = --mode ] && [ "$go2_test_argument" = approach ]; }; then
    export GO2_VISION_ROOT="${GO2_VISION_ROOT:-$go2_test_root}"
    if [ "$(git -C "$go2_test_root" branch --show-current)" != feature/map-approach-s1-grasp-integration ]; then
      if [ ! -f /tmp/go2_onekey_s1/scripts/go2_color_block_test.py ]; then
        echo '先更新 /tmp/go2_onekey_s1 集成分支；缺少方块接近入口。' >&2
        exit 1
      fi
      exec bash /tmp/go2_onekey_s1/go2_test.sh "$@"
    fi
    exec /usr/bin/python3 "$go2_test_root/scripts/go2_color_block_test.py" "$@"
  fi
  go2_test_previous="$go2_test_argument"
done
exec /usr/bin/python3 "$go2_test_root/scripts/go2_full_test.py" "$@"
