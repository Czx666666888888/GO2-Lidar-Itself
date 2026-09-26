#!/usr/bin/env bash
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IFACE="${1:-enp12s0}"
DOG_IP="${DOG_IP:-192.168.123.161}"
fail=0

if ! source "$ROOT/code/scripts/env_go2.sh" "$IFACE"; then exit 1; fi

echo "=== 网卡 ==="
ip -brief link show dev "$IFACE" 2>/dev/null || { echo "找不到网卡 $IFACE"; exit 1; }
ip -brief address show dev "$IFACE" 2>/dev/null || true
if ! ip -4 address show dev "$IFACE" 2>/dev/null | grep -q '192\.168\.123\.'; then
  echo "FAIL: $IFACE 没有 192.168.123.x/24 地址（建议电脑设为 192.168.123.99/24）"
  fail=1
fi

echo "=== GO2 连通性 ==="
if ping -c 1 -W 2 "$DOG_IP" >/dev/null 2>&1; then
  echo "OK: $DOG_IP 可达"
else
  echo "FAIL: ping 不到 $DOG_IP"
  fail=1
fi

echo "=== DDS 话题（最多等待 8 秒） ==="
topics="$(timeout 8 ros2 topic list 2>/dev/null || true)"
for topic in /lf/sportmodestate /api/sport/request; do
  if grep -qx "$topic" <<<"$topics"; then echo "OK: $topic"; else echo "MISS: $topic"; fail=1; fi
done

echo "=== UTLidar 实时数据 ==="
if ! GO2_ENV_READY=1 LIDAR_WAIT_SECONDS=8 \
    bash "$ROOT/code/scripts/go2_lidar_preflight.sh" "$IFACE"; then
  fail=1
fi

if ((fail)); then
  echo "预检未通过；不会发送任何运动指令。"
  exit 1
fi
echo "预检通过。建议先运行: bash '$ROOT/code/scripts/nav_launch.sh' --dry-run"
