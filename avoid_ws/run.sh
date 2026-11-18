#!/usr/bin/env bash
set -eo pipefail
workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
root=$(dirname "$workspace")
action=''
while (( $# )); do
  case "$1" in
    --mode)
      [[ "${2:-}" == avoid ]] || exit 2
      shift ;;
    --mode=avoid) ;;
    --dry-run|--observe|--execute|--stop)
      [[ -z "$action" ]] || { echo '只能指定一种测试操作' >&2; exit 2; }
      action="$1" ;;
    *)
      echo "未知参数 $1。用法：go2_test.sh --mode avoid [--dry-run|--observe|--execute|--stop]" >&2
      exit 2 ;;
  esac
  shift
done
action="${action:---dry-run}"
runtime="$workspace/log"
mkdir -p "$runtime"

if [[ "$action" == --dry-run ]]; then
  source /opt/ros/humble/setup.bash
  source "$root/deploy_ws/install/setup.bash"
else
  source "$root/code/scripts/env_go2.sh" "${GO2_IFACE:-enp12s0}"
fi

if [[ "$action" == --stop ]]; then
  if [[ -f "$runtime/motion.lock" ]] && ! flock -n "$runtime/motion.lock" true; then
    exec timeout 5 ros2 topic pub --once -w 1 /go2_reactive_avoid/stop std_msgs/msg/Int8 '{data: 1}'
  fi
  echo '没有正在运行的避障测试' >&2
  exit 1
fi

entry="$workspace/install/go2_reactive_avoid/lib/go2_reactive_avoid/avoid_test"
package="$workspace/src/go2_reactive_avoid"
if [[ ! -x "$entry" || "$package/setup.py" -nt "$entry" ||
      "$package/setup.cfg" -nt "$entry" || "$package/package.xml" -nt "$entry" ]]; then
  (
    flock 9
    cd "$workspace"
    colcon build --base-paths src --packages-select go2_reactive_avoid --symlink-install
  ) 9>"$runtime/build.lock"
fi
source "$workspace/install/local_setup.bash"

if [[ "$action" == --dry-run ]]; then
  /usr/bin/python3 -m unittest discover -s "$workspace/tests" -v
  "$entry" --help
  exit 0
fi

if [[ "$action" == --execute ]]; then
  exec 8>"$runtime/motion.lock"
  flock -n 8 || { echo '已有避障测试在运行' >&2; exit 1; }
fi
export ROS_LOG_DIR
ROS_LOG_DIR=$(mktemp -d "$runtime/run.XXXXXXXX")
echo "避障日志 $ROS_LOG_DIR"
set -- --verified-sport-writers "$workspace/sport_writers.local"
if [[ -n "${GO2_AVOID_LIDAR_HEIGHT:-}" ]]; then
  set -- "$@" --lidar-height "$GO2_AVOID_LIDAR_HEIGHT"
fi
exec "$entry" "${action#--}" "$@"
