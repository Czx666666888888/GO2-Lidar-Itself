# NX FULL 最终远程启动命令：可调前置速度与断开 PC 网线

已部署 NX 速度参数；以下命令会在人工确认后执行真实运动。示例为前置段目标速度 1.0 m/s、持续 3 秒，然后进入当前多目标 FULL。

## 当前电脑

```bash
ssh -t -i /home/czx666/.ssh/id_ed25519_go2 unitree@192.168.123.18 'python3 /home/unitree/go2_full_nx/go2_persistent_terminal.py -- bash /home/unitree/go2_full_nx/go2_nx.sh --execute --mode full --3 --1.0'
```

## 另一台电脑

有 SSH 客户端、能连接 `192.168.123.18`、具备 NX 账号登录权限即可；程序全部运行在 NX，无需在该电脑安装本项目或 ROS。有线地址需在 `192.168.123.0/24` 同网段且不与其他设备冲突。下面命令可在 Linux 终端或 Windows PowerShell 使用，登录时使用该电脑已有 SSH 密钥或按提示输入 NX 的 `unitree` 账户密码。

```bash
ssh -t unitree@192.168.123.18 'python3 /home/unitree/go2_full_nx/go2_persistent_terminal.py -- bash /home/unitree/go2_full_nx/go2_nx.sh --execute --mode full --3 --1.0'
```

## 确认和拔线

1. 等待 `NX_FULL_READY`。
2. 输入 `1` 并回车。
3. 看到 `NX_START_ACCEPTED` 后，可以拔 **PC 到 NX 的网线**。这表示确认已发送，任务终端由 NX 持有，不表示真机动作已成功。
4. 前置段按原规则在确认后等待 1 秒起步，IMU 保持初始航向；计时结束归零、停稳后进入 FULL。

NX 到 GO2 本体的连接以及相机等设备连接须保持。`Ctrl+C` 会请求停止，不能用它代替拔线。已有任务运行时，命令会重连该任务，新速度/时长不会覆盖正在运行的任务；新参数只对新任务生效。

重新连接网络后只查看原任务，不启动新任务：

```bash
ssh -t -i /home/czx666/.ssh/id_ed25519_go2 unitree@192.168.123.18 'python3 /home/unitree/go2_full_nx/go2_persistent_terminal.py --attach-only'
```

另一台电脑同样使用该命令，删去 `-i /home/czx666/.ssh/id_ed25519_go2`；如需指定密钥，使用那台电脑自己的密钥路径。日志位于 NX `/home/unitree/go2_full_nx/persistent_terminal/output.log`。

## 参数规则

| 参数 | 含义 |
| --- | --- |
| `--3` | 前置段持续 3 秒；整数后缀表示秒数 |
| `--1.0` | 前置段目标速度 1.0 m/s；小数后缀表示速度 |
| `--3 --0.6` | 3 秒，目标速度 0.6 m/s |
| `--timed-straight 2.5 --0.8` | 2.5 秒，目标速度 0.8 m/s |
| `--timed-speed 0.6` | 速度的完整参数写法，须同时指定时长 |

时长范围 `(0, 10]` 秒，速度范围 `[0.10, 1.00]` m/s。省略速度时仍为 1.0 m/s。原小数时长后缀 `--2.5` 现在代表速度，会因超过上限而报错；小数时长必须使用完整参数。重复指定时长、重复指定速度、无时长仅指定速度，以及非有限/越界数值均在启动前拒绝。

只改变前置段目标速度与对应 Sport gate 上限，起步渐加速、IMU 保持航向和到时归零停稳继续沿用原实现；不改 FULL 的接近速度、多目标、抓取侧一致性或接近时停止导航逻辑。

## 部署与证据

NX `/home/unitree/go2_full_nx/releases/599c86c/source` 已更新 `scripts/go2_full_test.py`、`scripts/nx_timed_options.py` 及 `scripts/nx_timed_move_only/go2_reactive_avoid/{node,timed_straight}.py`。备份在 `../timed_speed_backup.8pa6z7i2`。入口哈希 `ae62a649a111050b58520ef23c21b6ec8925364513a71428f4854b2407ac5baf`。完整哈希与差异见 [清单](go2_nx_timed_speed_manifest.json)、[补丁](go2_nx_timed_speed.patch)。本机准备/安装/测试脚本在 `scripts/nx_timed_speed/`。

本机和 NX 正式源码的 25 项离线测试通过。NX 正式 `go2_nx.sh` 对 `--3 --1.0` 及 `--timed-straight 2.5 --0.6` 两组离线计划解析通过。仅回环网络、隔离 DDS 域 81 的模拟测试中，Move 峰值分别为 0.6 和 1.0 m/s，时间结束发送零速并退出；0.6 m/s 场景的急停也通过。首次模拟因 Foxy 不支持较新 CycloneDDS Interfaces 配置而未创建节点，换成旧版 NetworkInterfaceAddress 配置后通过，未与真实 GO2 通信。

本轮没有实机运动、真实拔线或从另一台实体电脑登录验证；相应效果均 `NOT VERIFIED`。此前持久终端已通过模拟程序的断线继续和重连验证，本轮未修改该模块。
