#!/usr/bin/env bash
# 用法: source code/scripts/env_go2.sh [连接 GO2 的网卡名]
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "请用 source 执行此脚本: source $0 [网卡名]" >&2
  exit 2
fi

GO2_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
GO2_WS="$GO2_ROOT/deploy_ws"
GO2_IFACE="${1:-${GO2_IFACE:-enp12s0}}"

source /opt/ros/humble/setup.bash
if [[ ! -f "$GO2_WS/install/setup.bash" ]]; then
  echo "未找到编译产物，请先运行: bash '$GO2_ROOT/code/scripts/deploy_local.sh'" >&2
  return 1
fi
source "$GO2_WS/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
unset ROS_LOCALHOST_ONLY
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$GO2_IFACE\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
export GO2_ROOT GO2_WS GO2_IFACE

echo "[GO2] workspace=$GO2_WS"
echo "[GO2] interface=$GO2_IFACE domain=$ROS_DOMAIN_ID rmw=$RMW_IMPLEMENTATION"
