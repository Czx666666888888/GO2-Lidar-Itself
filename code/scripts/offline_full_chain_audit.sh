#!/usr/bin/env bash
# GO2 原始雷达 bag -> Point-LIO -> terrain -> WP5 -> planner 离线审计。
# 仅回放 cloud/IMU；不启动 safety_gate，不会发送 GO2 运动请求。
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BAG="${1:-$ROOT/rosbags/full_chain_a_retest5_20260901_160622}"
OUT="${2:-/tmp/go2_offline_chain_audit}"
DOMAIN="${OFFLINE_ROS_DOMAIN_ID:-196}"

source "$ROOT/code/scripts/env_go2.sh" lo >/dev/null || exit 2
export ROS_DOMAIN_ID="$DOMAIN"
unset ROS_LOCALHOST_ONLY
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" multicast="false"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'

mkdir -p "$OUT"
PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  sleep 2
  for pid in "${PIDS[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

ros2 launch vehicle_simulator system_real_robot_with_route_planner.launch rviz:=false \
  >"$OUT/nav.log" 2>&1 &
PIDS+=("$!")
ros2 run go2_keyboard_teleop base_odom_node >"$OUT/base.log" 2>&1 &
PIDS+=("$!")
ros2 run go2_keyboard_teleop wp5_explore_node >"$OUT/wp5.log" 2>&1 &
PIDS+=("$!")
sleep 12

ros2 bag play "$BAG" --clock --rate 1.0 \
  --topics /utlidar/cloud /utlidar/imu >"$OUT/bag.log" 2>&1 &
PIDS+=("$!")
sleep 35

echo '=== POINT CLOUD METRICS ==='
for topic in /utlidar/cloud /utlidar/transformed_cloud /registered_scan; do
  printf '%s points/frame: ' "$topic"
  timeout 8 ros2 topic echo "$topic" --qos-profile sensor_data --once \
    --field width 2>/dev/null | head -1 || echo 'NO_SAMPLE'
  rate="$(timeout 10 ros2 topic hz "$topic" --wall-time -w 20 2>/dev/null \
    | awk '/average rate/{line=$0} END{print line}')"
  echo "${rate:-average rate: NO_SAMPLE}"
done

ros2 topic pub /update_visibility_graph std_msgs/msg/Bool '{data: true}' --once \
  >/dev/null 2>&1 || true

fail=0
check_data() {
  local topic="$1"
  if timeout 8 ros2 topic echo "$topic" --qos-profile sensor_data --once \
      --no-arr >/dev/null 2>&1; then
    echo "PASS $topic"
  else
    echo "FAIL $topic"
    fail=1
  fi
}

echo '=== OFFLINE GO2 CHAIN ==='
for topic in \
  /utlidar/cloud /utlidar/imu \
  /utlidar/transformed_cloud /utlidar/transformed_imu \
  /state_estimation /base_state_estimation /registered_scan \
  /terrain_map /terrain_map_ext /way_point /path /cmd_vel_raw; do
  check_data "$topic"
done

# /goal_point 是事件型话题，可能在采样窗口内没有新消息；日志出现探索目标即通过。
if rg -q '\[EXPLORE\].*目标' "$OUT/wp5.log"; then
  echo 'PASS /goal_point (WP5 log evidence)'
else
  echo 'FAIL /goal_point'
  fail=1
fi

echo '=== CORE NODES ==='
ros2 node list 2>/dev/null | sort
echo '=== LOG ERRORS ==='
rg -n -i 'error|exception|failed|not found|segmentation|process has died' "$OUT"/*.log \
  | tail -80 || true
exit "$fail"
