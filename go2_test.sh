#!/usr/bin/env bash
# Unified entry; --time-diagnostic restores only the read-only localization chain.
# --nav-mode autonomous  launches full autonomous exploration + multi-target mission.
# --dry-run              skip real motion; check ROS graph only.
set -euo pipefail
go2_test_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# ── parse key flags early so autonomous mode can intercept before the rest ──
_go2_nav_mode=''
_go2_detector='yolo'
_go2_dry_run=0
_go2_execute=0
_go2_prev=''
for _a in "$@"; do
  case "$_a" in
    --nav-mode=autonomous) _go2_nav_mode=autonomous ;;
    --nav-mode=straight)   _go2_nav_mode=straight ;;
    --detector=*)          _go2_detector="${_a#--detector=}" ;;
    --dry-run)             _go2_dry_run=1 ;;
    --execute)             _go2_execute=1 ;;
  esac
  if [ "$_go2_prev" = --nav-mode ]; then _go2_nav_mode="$_a"; fi
  if [ "$_go2_prev" = --detector ]; then _go2_detector="$_a"; fi
  _go2_prev="$_a"
done

# ── autonomous nav mode ───────────────────────────────────────────────────────
if [ "$_go2_nav_mode" = autonomous ]; then
  export GO2_VISION_ROOT="${GO2_VISION_ROOT:-$go2_test_root}"
  _go2_overlay="${go2_test_root}/camera_ws/install/go2_science_perception/setup.bash"
  _go2_deploy="${go2_test_root}/deploy_ws/install/setup.bash"

  if [ "$_go2_dry_run" = 1 ] || [ "$_go2_execute" = 0 ]; then
    echo "自主导航模式（干运行/离线计划）"
    echo "  将启动以下节点组合："
    echo "  1. Point-LIO SLAM + terrain_analysis + terrain_analysis_ext"
    echo "  2. FAR全局规划 (localPlanner autonomyMode=true maxSpeed=0.4)"
    echo "  3. D435 + YOLO目标检测 / science_target_manager"
    echo "  4. MotionArbiter (唯一/cmd_vel_raw发布者)"
    echo "  5. ScienceTargetApproach (remap到/science/cmd_vel_raw)"
    echo "  6. MissionManager (探索→导航→接近→抓取循环)"
    echo "  7. go2_safety_gate (仅--execute模式)"
    echo ""
    echo "真实执行: bash go2_test.sh --execute --mode full --nav-mode autonomous --detector ${_go2_detector}"
    echo "接口检查: bash go2_test.sh --dry-run --mode full --nav-mode autonomous"
    exit 0
  fi

  # 时钟检查
  if [ "${GO2_PC_TIME_HELD:-}" != 1 ] && [ "${GO2_RUNTIME_PROFILE:-}" != local ] && [ "$(uname -m)" != aarch64 ]; then
    exec /usr/bin/python3 -u "$go2_test_root/camera_ws/scripts/pc_time_check.py" "$@"
  fi

  exec /usr/bin/python3 "$go2_test_root/scripts/go2_autonomous_mission.py" \
    --detector "$_go2_detector" \
    --vision-root "$go2_test_root" \
    --overlay "$_go2_overlay" \
    --deploy-ws "$_go2_deploy" \
    "$@"
fi

# ── existing approach / vision-only mode ─────────────────────────────────────
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
if [ "${1:-}" = --time-diagnostic ]; then
  shift
  exec bash "$go2_test_root/camera_ws/scripts/run_go2_time_diagnostic.sh" "$@"
fi
# Guard PC time before handing off to the independently updated integration clone.
if [ "${1:-}" = --execute ] && [ "${GO2_PC_TIME_HELD:-}" != 1 ] && [ "${GO2_RUNTIME_PROFILE:-}" != local ] && [ "$(uname -m)" != aarch64 ]; then
  exec /usr/bin/python3 -u "$go2_test_root/camera_ws/scripts/pc_time_check.py" "$@"
fi
if [ "${1:-}" = --execute ] && [ "$(git -C "$go2_test_root" branch --show-current)" != feature/map-approach-s1-grasp-integration ]; then
  go2_test_integration=/tmp/go2_onekey_s1
  if [ ! -e "$go2_test_integration" ]; then
    git clone --branch feature/map-approach-s1-grasp-integration --single-branch \
      https://github.com/Czx666666888888/GO2-Lidar-Itself.git "$go2_test_integration"
  fi
  if [ ! -d "$go2_test_integration/.git" ] || [ ! -f "$go2_test_integration/go2_test.sh" ]; then
    echo 'S1集成目录不完整，保留现场并停止。' >&2
    exit 1
  fi
  if [ "$(git -C "$go2_test_integration" branch --show-current)" != feature/map-approach-s1-grasp-integration ]; then
    echo 'S1集成目录分支不符，保留现场并停止。' >&2
    exit 1
  fi
  mkdir -p "$go2_test_root/camera_ws/records"
  for go2_test_link in deploy_ws camera_ws/records; do
    go2_test_target="$go2_test_root/$go2_test_link"
    go2_test_dest="$go2_test_integration/$go2_test_link"
    if [ ! -e "$go2_test_dest" ] && [ ! -L "$go2_test_dest" ]; then
      ln -s "$go2_test_target" "$go2_test_dest"
    fi
    if [ ! -L "$go2_test_dest" ] || [ "$(readlink -f "$go2_test_dest")" != "$(readlink -f "$go2_test_target")" ]; then
      echo "集成目录链接不符，保留现场并停止：$go2_test_dest" >&2
      exit 1
    fi
  done
  exec bash "$go2_test_integration/go2_test.sh" "$@"
fi
exec /usr/bin/python3 "$go2_test_root/scripts/go2_full_test.py" "$@"
