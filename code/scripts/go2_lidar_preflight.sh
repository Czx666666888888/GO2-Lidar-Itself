#!/usr/bin/env bash
# Validate the native GO2 UTLidar DDS feed.  No PC-side LiDAR driver is needed:
# the robot publishes /utlidar/cloud and /utlidar/imu directly over CycloneDDS.
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IFACE="${1:-${GO2_IFACE:-enp12s0}}"
WAIT_SECONDS="${LIDAR_WAIT_SECONDS:-12}"

if [[ "${GO2_ENV_READY:-0}" != "1" ]]; then
  source "$ROOT/code/scripts/env_go2.sh" "$IFACE" || exit 1
fi

fail=0
check_stream() {
  local topic="$1"
  local expected_type="$2"
  local actual_type

  actual_type="$(timeout "$WAIT_SECONDS" ros2 topic type "$topic" 2>/dev/null | head -1)"
  if [[ "$actual_type" != "$expected_type" ]]; then
    echo "FAIL: $topic 类型异常（期望 $expected_type，实际 ${actual_type:-未发现}）" >&2
    fail=1
    return
  fi

  if timeout "$WAIT_SECONDS" ros2 topic echo "$topic" "$expected_type" \
      --qos-profile sensor_data --once --no-arr >/dev/null 2>&1; then
    echo "OK: $topic ($expected_type) 有实时数据"
  else
    echo "FAIL: $topic 已发现，但 ${WAIT_SECONDS}s 内没有数据" >&2
    fail=1
  fi
}

echo "=== GO2 原生 DDS 雷达接入（网卡: $IFACE） ==="
check_stream /utlidar/cloud sensor_msgs/msg/PointCloud2
check_stream /utlidar/imu sensor_msgs/msg/Imu

if ((fail)); then
  cat >&2 <<EOF
雷达接入未通过。请确认：
  1. 电脑网卡 $IFACE 位于 192.168.123.0/24；
  2. GO2 机载雷达服务已启用；
  3. ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} 与机器人一致；
  4. 没有设置 ROS_LOCALHOST_ONLY=1。
不会启动建图或发送运动指令。
EOF
  exit 1
fi

echo "雷达接入通过：可启动 transform_sensors 与 Point-LIO。"
