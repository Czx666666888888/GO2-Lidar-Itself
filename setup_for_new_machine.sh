#!/usr/bin/env bash
# 在新机器上首次运行此脚本，完成机器特定配置的修改。
# 用法：bash setup_for_new_machine.sh
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

echo "=== GO2 自主任务系统 — 新机器初始化 ==="
echo ""

# ── 1. 网卡名 ─────────────────────────────────────────────────────────────────
echo "当前网卡列表："
ip -br link show | grep -v lo
echo ""
read -rp "请输入与 GO2 直连的有线网卡名（例如 enp12s0 / eth0）：" NIC
if ! ip link show "$NIC" &>/dev/null; then
  echo "错误：找不到网卡 $NIC" >&2; exit 1
fi

# ── 2. PC IP ──────────────────────────────────────────────────────────────────
PC_IP="$(ip -4 addr show "$NIC" | grep -oP '(?<=inet )[0-9.]+' | head -1 || true)"
echo "检测到 $NIC 当前 IP：${PC_IP:-未配置}"
read -rp "PC 有线 IP（192.168.123.x 段，直接回车使用 192.168.123.99）：" INPUT_IP
PC_IP="${INPUT_IP:-192.168.123.99}"

# ── 3. GO2 IP ─────────────────────────────────────────────────────────────────
read -rp "GO2 IP（直接回车使用 192.168.123.18）：" INPUT_GO2_IP
GO2_IP="${INPUT_GO2_IP:-192.168.123.18}"
GO2_HOST="unitree@${GO2_IP}"

# ── 4. SSH 密钥 ───────────────────────────────────────────────────────────────
DEFAULT_KEY="$HOME/.ssh/id_ed25519_go2"
read -rp "SSH 私钥路径（直接回车使用 ${DEFAULT_KEY}）：" INPUT_KEY
SSH_KEY="${INPUT_KEY:-$DEFAULT_KEY}"
if [ ! -f "$SSH_KEY" ]; then
  echo "密钥文件不存在，自动生成：$SSH_KEY"
  ssh-keygen -t ed25519 -f "$SSH_KEY" -N ""
  echo ""
  echo "公钥已生成，请将以下公钥添加到 GO2（unitree@${GO2_IP}）的 ~/.ssh/authorized_keys："
  cat "${SSH_KEY}.pub"
  echo ""
  read -rp "完成后按回车继续..."
fi

# ── 5. D435 序列号 ────────────────────────────────────────────────────────────
echo ""
echo "正在扫描已连接的 RealSense 设备..."
SERIAL="$(python3 -c "
import pyrealsense2 as r
devs = r.context().query_devices()
serials = [d.get_info(r.camera_info.serial_number) for d in devs]
print(' '.join(serials))
" 2>/dev/null || true)"
if [ -n "$SERIAL" ]; then
  echo "检测到设备序列号：$SERIAL"
  read -rp "使用哪个序列号（直接回车使用第一个）：" INPUT_SERIAL
  SERIAL="${INPUT_SERIAL:-${SERIAL%% *}}"
else
  echo "未自动检测到设备，请手动查看 rs-enumerate-devices 输出。"
  read -rp "请输入 D435 序列号：" SERIAL
fi

echo ""
echo "── 配置摘要 ──────────────────────────────────────────────"
echo "  网卡：       $NIC"
echo "  PC IP：      $PC_IP"
echo "  GO2 IP：     $GO2_IP"
echo "  SSH 密钥：   $SSH_KEY"
echo "  D435 序列号：$SERIAL"
echo "──────────────────────────────────────────────────────────"
read -rp "确认修改？[y/N] " CONFIRM
if [[ "${CONFIRM,,}" != "y" ]]; then
  echo "已取消。"; exit 0
fi

# ── 6. 应用修改 ───────────────────────────────────────────────────────────────

# go2_full_test.py
FT="$ROOT/scripts/go2_full_test.py"
sed -i \
  "s|HOST = 'unitree@[^']*'|HOST = '$GO2_HOST'|g" \
  "$FT"
sed -i \
  "s|KEY = '[^']*'|KEY = '$SSH_KEY'|g" \
  "$FT"
sed -i \
  "s|GO2_D435_SERIAL='[0-9]*'|GO2_D435_SERIAL='$SERIAL'|g" \
  "$FT"
echo "已修改：$FT"

# go2_autonomous_mission.py（继承 go2_full_test 参数，无需单独修改，但确认 KEY 路径）
AM="$ROOT/scripts/go2_autonomous_mission.py"
if [ -f "$AM" ]; then
  sed -i \
    "s|/home/czx666/.ssh/id_ed25519_go2|$SSH_KEY|g" \
    "$AM"
  echo "已修改：$AM"
fi

# go2_test.sh — 路由检查里的网卡名和 IP
GT="$ROOT/go2_test.sh"
sed -i \
  "s|enp12s0|$NIC|g" \
  "$GT"
sed -i \
  "s|192.168.123.99|$PC_IP|g" \
  "$GT"
echo "已修改：$GT"

# run_map_s1_task.py（如果存在）
RMS="$ROOT/camera_ws/scripts/run_map_s1_task.py"
if [ -f "$RMS" ]; then
  sed -i \
    "s|unitree@192.168.123.18|$GO2_HOST|g" \
    "$RMS"
  sed -i \
    "s|/home/czx666/.ssh/id_ed25519_go2|$SSH_KEY|g" \
    "$RMS"
  echo "已修改：$RMS"
fi

# ── 7. yolo_detector.yaml — 写入 model_path ──────────────────────────────────
MODEL_PT="$ROOT/vision_training/runs/science_targets_mixed_v3/baseline/weights/best.pt"
YAML="$ROOT/camera_ws/src/go2_science_perception/config/yolo_detector.yaml"
if [ -f "$MODEL_PT" ] && [ -f "$YAML" ]; then
  # 替换 model_path: '' 为实际路径（只替换空值，避免覆盖已有配置）
  sed -i \
    "s|model_path: ''|model_path: '$MODEL_PT'|g" \
    "$YAML"
  echo "已写入 model_path：$YAML"
else
  echo "警告：模型文件或 yaml 不存在，请手动设置 model_path。"
fi

# ── 8. 验证 SSH ───────────────────────────────────────────────────────────────
echo ""
echo "正在验证 SSH 连接到 $GO2_HOST ..."
if ssh -i "$SSH_KEY" -o BatchMode=yes -o ConnectTimeout=5 \
     "$GO2_HOST" echo "SSH_OK" 2>/dev/null; then
  echo "SSH 连接成功。"
else
  echo "警告：SSH 连接失败。请确认："
  echo "  1. GO2 已开机且有线连接正常"
  echo "  2. 公钥已添加到 GO2 的 ~/.ssh/authorized_keys"
  echo "  3. 网卡 IP 已配置为 $PC_IP"
fi

echo ""
echo "=== 初始化完成 ==="
echo "下一步："
echo "  1. 按 FINAL.md 步骤 2-4 重新编译 deploy_ws 和感知包"
echo "  2. 验证接口：bash go2_test.sh --mode full --nav-mode autonomous --dry-run"
echo "  3. 真实执行：bash go2_test.sh --execute --mode full --nav-mode autonomous"
