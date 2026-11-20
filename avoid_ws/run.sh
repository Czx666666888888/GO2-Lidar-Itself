#!/usr/bin/env bash
set -eo pipefail
workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
root=$(dirname "$workspace")
action=''
entrance=()
while (( $# )); do
  case "$1" in
    --mode)
      [[ "${2:-}" == avoid ]] || exit 2
      shift ;;
    --mode=avoid) ;;
    --entrance)
      [[ $# -ge 3 && ${#entrance[@]} -eq 0 ]] || exit 2
      entrance=("$1" "$2" "$3")
      shift 2 ;;
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
if (( ${#entrance[@]} )) && [[ "$action" != --execute ]]; then
  exit 2
fi
runtime="$workspace/log"
mkdir -p "$runtime"
echo "避障入口 $action，正在加载 ROS 环境"
echo "最小规则：0.20 m/s 直行；前25cm停车转向；单侧15cm反向20°弧线；0=未检出，1=检出"

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

if [[ "$action" != --dry-run ]]; then
  if [[ ! -e "/sys/class/net/$GO2_IFACE" ]]; then
    echo "GO2 网卡 $GO2_IFACE 不存在" >&2
    exit 1
  fi
  if ! read -r carrier 2>/dev/null < "/sys/class/net/$GO2_IFACE/carrier" || [[ "$carrier" != 1 ]]; then
    echo "GO2 网卡 $GO2_IFACE 无链路（NO-CARRIER），未启动 DDS" >&2
    exit 1
  fi
fi

entry="$workspace/install/go2_reactive_avoid/lib/go2_reactive_avoid/avoid_test"
package="$workspace/src/go2_reactive_avoid"
(
  echo "等待避障构建锁（最多 30 秒） $runtime/build.lock"
  if ! flock -w 30 9; then
    echo "无法取得避障构建锁，退出。请查看占用该锁的构建进程 $runtime/build.lock" >&2
    exit 75
  fi
  if [[ ! -x "$entry" || "$package/setup.py" -nt "$entry" ||
        "$package/setup.cfg" -nt "$entry" || "$package/package.xml" -nt "$entry" ]]; then
    echo '已取得构建锁，开始构建 go2_reactive_avoid（最多 300 秒）'
    cd "$workspace"
    status=0
    timeout --signal=INT --kill-after=10 300 colcon build --base-paths src \
      --packages-select go2_reactive_avoid --symlink-install --event-handlers console_direct+ || status=$?
    if (( status != 0 )); then
      echo "避障构建失败，退出码 $status，日志 $runtime/latest_build" >&2
      exit "$status"
    fi
    echo '避障构建完成'
  else
    echo '已取得构建锁，复用避障构建产物'
  fi
) 9>"$runtime/build.lock"
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
writers="$workspace/sport_writers.local"
if [[ "$action" == --execute ]]; then
  echo '启动前只读核对 Sport 端点（最长 12 秒）'
  if ! timeout --signal=INT --kill-after=2 12 /usr/bin/python3 \
      -m go2_reactive_avoid.writer_audit "$writers" "$ROS_LOG_DIR/sport_writers.trace"; then
    echo "Sport 端点复核未完成，避障退出。日志 $ROS_LOG_DIR" >&2
    exit 1
  fi
fi
set -- --verified-sport-writers "$writers" "${entrance[@]}"
if [[ -n "${GO2_AVOID_LIDAR_HEIGHT:-}" ]]; then
  set -- "$@" --lidar-height "$GO2_AVOID_LIDAR_HEIGHT"
fi
exec "$entry" "${action#--}" "$@"
