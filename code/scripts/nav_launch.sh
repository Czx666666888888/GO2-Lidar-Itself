#!/bin/bash
# ============================================================
# Go2 自主导航完整启动脚本 (建图 + 探索 + 规划 + 跟踪 + 安全门 + 执行)
#
# 命令链 (WP5 在线化后):
#   狗传感器(/utlidar/cloud, /utlidar/imu)                 [狗端, DDS 已连]
#     → transform_sensors → Point-LIO → /state_estimation (位姿)
#     → terrain_analysis → /terrain_map (地形图, 含 unknown=1.5)
#     → wp5_explore_node → /goal_point (前沿观察点)
#     → far_planner → /way_point (全局路径点)
#     → localPlanner → /path (局部路径)
#     → pathFollower → /cmd_vel_raw (速度指令)
#     → go2_safety_gate → /api/sport/request (Move/Stop) → 狗
#       ↑ 狗位置(/state_estimation) → wp5 监视到达 → 重选 → 闭环
#
# 用法:
#   bash scripts/nav_launch.sh              # 默认 dry-run，狗不动
#   bash scripts/nav_launch.sh --real       # 实机模式，允许下发运动指令
#   bash scripts/nav_launch.sh --no-arm     # 不自动武装, 手动 arm
#
# 停止: 按 Ctrl-C (自动清理所有节点), 或 bash scripts/nav_stop.sh
# ============================================================
set -o pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
GO2_IFACE="${GO2_IFACE:-enp12s0}"
source "$ROOT/code/scripts/env_go2.sh" "$GO2_IFACE" || exit 1

# 禁止重复启动控制链，避免多个安全门/规划器同时运行。
# 锚定真实可执行命令，避免 shell/沙箱父进程携带检测字符串时被误判。
if pgrep -f '^(/usr/bin/python3 )?.*/(go2_safety_gate|pointlio_mapping|pathFollower|wp5_explore_node)( |$)' >/dev/null 2>&1; then
  echo "[错误] 检测到已有 GO2 导航进程，拒绝重复启动。" >&2
  echo "       请先运行: bash '$ROOT/code/scripts/nav_stop.sh'" >&2
  exit 1
fi

# ---------- 可调参数 ----------
DOG_IP="${DOG_IP:-192.168.123.161}"
NAV_LAUNCH="vehicle_simulator system_real_robot_with_route_planner.launch"
GATE_MAX_SPEED="${GATE_MAX_SPEED:-0.20}"       # 首次实机测试保守上限
GATE_MAX_YAWRATE="${GATE_MAX_YAWRATE:-0.30}"
WP5_ARRIVE_DIS=0.35
WP5_GOAL_TIMEOUT=60.0
ARM_AUTO=1                # 1=自动武装, 0=手动
DRY_RUN="true"            # 默认安全；显式 --real 才允许下发运动指令
OUTDIR="$ROOT/tmp/nav_run"
# ------------------------------

# 参数解析
for a in "$@"; do
  case "$a" in
    --dry-run) DRY_RUN="true" ;;
    --real)    DRY_RUN="false" ;;
    --no-arm)  ARM_AUTO=0 ;;
  esac
done

mkdir -p "$OUTDIR"
NAV_LOG="$OUTDIR/nav_chain.log"
GATE_LOG="$OUTDIR/safety_gate.log"
WP5_LOG="$OUTDIR/wp5_explore.log"

log()  { echo -e "\e[1;36m[启动]\e[0m $*"; }
warn() { echo -e "\e[1;33m[警告]\e[0m $*"; }
err()  { echo -e "\e[1;31m[错误]\e[0m $*"; }

# ---------- 清理 (全程覆盖: 启动阶段/运行阶段 Ctrl-C 都能清理) ----------
cleanup() {
  echo ""
  log ">>> 清理所有节点..."
  [ -n "$NAVPID"  ] && kill -INT -- -"$NAVPID"  2>/dev/null
  [ -n "$BASEPID" ] && kill -INT -- -"$BASEPID" 2>/dev/null
  [ -n "$GATEPID" ] && kill -INT -- -"$GATEPID" 2>/dev/null
  [ -n "$WP5PID"  ] && kill -INT -- -"$WP5PID"  2>/dev/null
  sleep 2
  # 兜底: 按可执行名精确清理 (防止 ros2 run 子进程残留)
  pkill -INT -f "pointlio_mapping"    2>/dev/null
  pkill -INT -f "transform_everything" 2>/dev/null
  pkill -INT -f "localPlanner"         2>/dev/null
  pkill -INT -f "pathFollower"         2>/dev/null
  pkill -INT -f "terrainAnalysis"      2>/dev/null
  pkill -INT -f "terrainAnalysisExt"   2>/dev/null
  pkill -INT -f "far_planner"          2>/dev/null
  pkill -INT -f "graph_decoder"        2>/dev/null
  pkill -INT -f "go2_safety_gate"      2>/dev/null
  pkill -INT -f "wp5_explore_node"     2>/dev/null
  pkill -INT -f "base_odom_node"       2>/dev/null
  pkill -INT -f "rviz2.*far_planner/rviz/default.rviz" 2>/dev/null
  # 无论是否武装, 直接发 StopMove(1003) 让狗停
  ros2 topic pub /api/sport/request unitree_api/msg/Request \
      "{header: {identity: {api_id: 1003}}}" --once >/dev/null 2>&1 || true
  log ">>> 已停止"
}
trap cleanup INT TERM EXIT

echo "=============================================================="
echo " Go2 自主导航完整启动   (dry_run=$DRY_RUN  auto_arm=$ARM_AUTO)"
echo "=============================================================="

# ---------- 0. 预检: 网络 + 狗传感器 ----------
log "[0] 预检: 网络与狗传感器"
if ! ping -c1 -W2 "$DOG_IP" >/dev/null 2>&1; then
  err "ping 不到狗 ($DOG_IP)。请检查网线/网卡 $GO2_IFACE, 然后重跑。"
  exit 1
fi
echo "    ✓ 狗可达 ($DOG_IP)"

# GO2 机载 UTLidar 直接通过 DDS 发布，不需要电脑端 go2_robot_sdk。
# cloud/IMU 任一缺失都立即退出，避免导航链在空数据上假启动。
if ! GO2_ENV_READY=1 LIDAR_WAIT_SECONDS=15 \
    bash "$ROOT/code/scripts/go2_lidar_preflight.sh" "$GO2_IFACE"; then
  err "GO2 雷达 DDS 接入失败，停止启动。"
  exit 1
fi

# ---------- 0.5. 机身中心里程计转换 (原始雷达位姿 → 机身中心, 供 FAR 用) ----------
log "[0.5] 启动机身中心里程计转换 (base_odom_node)"
setsid ros2 run go2_keyboard_teleop base_odom_node --ros-args \
    -p sensor_offset_x:=0.3 -p sensor_offset_y:=0.0 \
    > "$OUTDIR/base_odom.log" 2>&1 &
BASEPID=$!
echo "    PID=$BASEPID 日志=$OUTDIR/base_odom.log"
sleep 2

# ---------- 1. 导航链 (transform_sensors + Point-LIO + terrain + local/far planner) ----------
log "[1] 启动导航链: $NAV_LAUNCH"
setsid ros2 launch $NAV_LAUNCH > "$NAV_LOG" 2>&1 &
NAVPID=$!
echo "    PID=$NAVPID 日志=$NAV_LOG"
sleep 15

# ---------- 2. 安全门 (唯一 /api/sport/request 发布者) ----------
log "[2] 启动安全门 (max_speed=$GATE_MAX_SPEED max_yaw=$GATE_MAX_YAWRATE)"
setsid ros2 run go2_keyboard_teleop go2_safety_gate --ros-args \
    -p dry_run:="$DRY_RUN" \
    -p max_speed:="$GATE_MAX_SPEED" \
    -p max_yaw_rate:="$GATE_MAX_YAWRATE" \
    > "$GATE_LOG" 2>&1 &
GATEPID=$!
echo "    PID=$GATEPID 日志=$GATE_LOG"
sleep 3

# 真机模式必须先取得 Unitree Sport 控制权，禁止“已发武装但实际无控制权”。
if [ "$DRY_RUN" = "false" ]; then
  if ! grep -q "Sport lease 已取得" "$GATE_LOG"; then
    err "安全门未取得 Sport lease，拒绝继续。查看: $GATE_LOG"
    exit 1
  fi
  echo "    ✓ Sport lease 已取得"
fi

# ---------- 3. WP5 在线探索 (自动选点+发目标+监视到达+重选) ----------
log "[3] 启动 WP5 在线探索 (arrive=$WP5_ARRIVE_DIS m, timeout=$WP5_GOAL_TIMEOUT s)"
setsid ros2 run go2_keyboard_teleop wp5_explore_node --ros-args \
    -p arrive_dis:="$WP5_ARRIVE_DIS" \
    -p goal_timeout:="$WP5_GOAL_TIMEOUT" \
    > "$WP5_LOG" 2>&1 &
WP5PID=$!
echo "    PID=$WP5PID 日志=$WP5_LOG"
sleep 3

# ---------- 4. 等狗状态 → 武装安全门 ----------
if [ "$ARM_AUTO" = "1" ]; then
  log "[4] 等狗状态(/lf/sportmodestate)并武装"
  for i in $(seq 1 10); do
    if timeout 2 ros2 topic echo /lf/sportmodestate --once >/dev/null 2>&1; then
      echo "    ✓ 狗状态就绪 (第 ${i}s)"
      ros2 topic pub /arm std_msgs/Bool "{data: true}" --once >/dev/null 2>&1
      sleep 1
      if ! grep -q -- "-> ARMED" "$GATE_LOG"; then
        err "已发 /arm=true，但安全门未进入 ARMED。查看: $GATE_LOG"
        exit 1
      fi
      echo "    ✓ 安全门已确认进入 ARMED"
      break
    fi
    sleep 1
  done
else
  warn "跳过自动武装(--no-arm)。手动: ros2 topic pub /arm std_msgs/Bool \"{data: true}\" --once"
fi

# ---------- 5. 触发 far_planner 更新可视度图 ----------
log "[5] 触发 far_planner (/update_visibility_graph=true)"
ros2 topic pub /update_visibility_graph std_msgs/Bool "{data: true}" --once >/dev/null 2>&1

# ---------- 6. 快速自检: /goal_point 是否已选出真实目标(非 0,0) ----------
log "[6] 自检 /goal_point (应选到非零前沿目标)"
GX=""; GY=""
for i in $(seq 1 20); do
  LINE=$(timeout 3 ros2 topic echo /goal_point --once 2>/dev/null | grep -E "^  x:|^  y:" | head -2)
  GX=$(echo "$LINE" | awk '/x:/{print $2}' | head -1)
  GY=$(echo "$LINE" | awk '/y:/{print $2}' | head -1)
  if [ -n "$GX" ] && { [ "$GX" != "0.0" ] || [ "$GY" != "0.0" ]; }; then
    echo "    ✓ WP5 已选目标 ($GX, $GY) (探索闭环开始)"
    break
  fi
  sleep 2
done
if [ -z "$GX" ]; then
  warn "没收到 /goal_point。查: wp5 是否起来($WP5_LOG)、地形图是否出数据(bash $ROOT/code/scripts/check_chain.sh)"
elif [ "$GX" = "0.0" ] && [ "$GY" = "0.0" ]; then
  warn "WP5 在返航 home(0,0): 地图暂未扩出前沿。等建图出前沿即可(不是起点不可达)。"
fi

# ---------- 运行中 ----------
echo ""
echo "=============================================================="
echo " 自主导航链运行中 (按 Ctrl-C 停止并清理)"
if [ "$DRY_RUN" = "true" ]; then
  echo " 模式: DRY-RUN —— 狗不会动, 只验证链路"
fi
echo " 闭环: wp5选点 → far_planner → localPlanner → pathFollower → 安全门 → 狗 → 到达重选"
echo " 日志:"
echo "   导航链   $NAV_LOG"
echo "   安全门   $GATE_LOG"
echo "   探索     $WP5_LOG"
echo " 实时观察:"
echo "   ros2 topic echo /goal_point          (探索目标)"
echo "   ros2 topic echo /way_point           (规划路径点)"
echo "   ros2 topic echo /cmd_vel_raw         (速度指令)"
echo "   ros2 topic echo /api/sport/request   (狗运动指令)"
echo "   bash $ROOT/code/scripts/check_chain.sh  (逐环体检)"
  echo " 急停: ros2 topic pub /stop std_msgs/Int8 \"{data: 1}\" --once"
echo "=============================================================="

wait $NAVPID 2>/dev/null
