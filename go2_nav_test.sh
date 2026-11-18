#!/usr/bin/env bash
# Standalone navigation test. Run from any directory.
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export NAV_TEST_ROOT="$root/navigation_test"
runtime="$root/tmp/navigation_test"
export NAV_FOLLOWER="$runtime/follower/pathFollower"
mode="${1:---dry-run}"
view=false
if [[ "${2:-}" == --view && ( "$mode" == --execute || "$mode" == --dry-run ) ]]; then
  view=true
elif (( $# > 1 )); then
  echo '--view 只能跟在 --execute 或 --dry-run 后面' >&2
  exit 2
fi
if (( $# > 2 )); then echo '参数过多' >&2; exit 2; fi
case "$mode" in
  --execute|--dry-run|--return|--stop) ;;
  *) echo '用法: go2_nav_test.sh [--dry-run|--execute|--return|--stop] [--view]' >&2; exit 2 ;;
esac
source "$root/code/scripts/env_go2.sh" "${GO2_IFACE:-enp12s0}"
# This standalone chain starts fourteen DDS participants.
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$GO2_IFACE\"/></Interfaces></General><Discovery><MaxAutoParticipantIndex>64</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"
if [[ "$mode" == --return || "$mode" == --stop ]]; then
  if [[ -e "$runtime/active.lock" ]] && ! flock -n "$runtime/active.lock" true; then
    if [[ "$mode" == --return ]]; then
      exec timeout 5 ros2 topic pub --once -w 1 /navigation_test/return_home std_msgs/msg/Bool '{data: true}'
    fi
    exec timeout 5 ros2 topic pub --once -w 1 /navigation_test/stop std_msgs/msg/Int8 '{data: 1}'
  fi
  echo '没有活动的独立导航测试' >&2; exit 1
fi
mkdir -p "$runtime"
exec 9>"$runtime/build.lock"
flock 9
cmake -S "$NAV_TEST_ROOT" -B "$runtime/follower" -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build "$runtime/follower"
flock -u 9
export PYTHONPATH="$root/code/go2_keyboard_teleop:${PYTHONPATH:-}"
export ROS_LOG_DIR
ROS_LOG_DIR=$(mktemp -d "$runtime/run.XXXXXXXX")
export NAV_RESULT="$ROS_LOG_DIR/result.txt"
if [[ "$mode" == --dry-run ]]; then
  /usr/bin/python3 "$NAV_TEST_ROOT/tests/test_navigation_regressions.py"
  ros2 launch "$NAV_TEST_ROOT/navigation.launch.py" "view:=$view" --show-args
  echo '独立导航离线检查通过'
  exit 0
fi
exec 8>"$runtime/active.lock"
flock -n 8 || { echo '已有导航测试在运行' >&2; exit 1; }
nodes=$(timeout 8 ros2 node list --no-daemon --spin-time 2)
if grep -Eq '/(laserMapping|transform_everything|sensor_transformer|go2_safety_gate|navigation_test_gate|mission_manager|pathFollower|localPlanner|motion_arbiter|nbv_explore_node|far_planner)$' <<< "$nodes"; then
  echo '检测到已有定位、导航或控制节点，结束本次启动' >&2; exit 1
fi
export NAV_EXECUTE=1
echo "导航日志 $ROS_LOG_DIR"
ros2 launch "$NAV_TEST_ROOT/navigation.launch.py" "view:=$view"
[[ -s "$NAV_RESULT" ]] && cat "$NAV_RESULT"
[[ -s "$NAV_RESULT" && "$(< "$NAV_RESULT")" == DONE ]]
