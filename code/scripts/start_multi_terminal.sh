#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ENV_CMD="cd '$ROOT'; source code/scripts/env_go2.sh enp12s0"

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "未找到 gnome-terminal，无法自动打开多终端。" >&2
  exit 1
fi

gnome-terminal --window --title="GO2导航主链" -- bash -lc \
  "$ENV_CMD; GATE_MAX_SPEED=0.20 GATE_MAX_YAWRATE=0.50 bash code/scripts/nav_launch.sh --real --no-arm; exec bash"
gnome-terminal --window --title="GO2链路检查" -- bash -lc \
  "$ENV_CMD; echo '等待导航链完全启动 (45s)...'; sleep 45; bash code/scripts/check_chain.sh; exec bash"
gnome-terminal --window --title="GO2实时日志" -- bash -lc \
  "$ENV_CMD; mkdir -p tmp/nav_run; touch tmp/nav_run/safety_gate.log tmp/nav_run/wp5_explore.log; tail -F tmp/nav_run/safety_gate.log tmp/nav_run/wp5_explore.log; exec bash"
gnome-terminal --window --title="GO2控制台" -- bash -lc \
  "$ENV_CMD; printf '\nGO2 控制台（当前未武装）\n\n武装： ros2 topic pub /arm std_msgs/msg/Bool \"{data: true}\" --once\n急停： ros2 topic pub /stop std_msgs/msg/Int8 \"{data: 1}\" --once\n停止： bash code/scripts/nav_stop.sh\n\n'; exec bash"

echo "GO2 多终端已启动（真机链路，未武装）。"
