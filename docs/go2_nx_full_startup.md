# NX 自主 FULL 启动与人工确认

## NX FULL 最新接收数据模式

NX 入口通过 `GO2_LATEST_DATA=1` 启用最新输入模式。导航、接近和目标转换查询最新可用 TF，里程计使用最近收到的样本。源时间戳过旧、重复、回退、未来或为零不会再触发原应用层匹配拒绝。RGB-D 使用最近接收的 RGB、深度及相机内参，取消 RGB/深度/内参之间的源时间差门槛；HSV 模式也取消 RGB-D 时间差门槛。目标观测按处理时刻进入原关联、去重及到位流程。原始 YOLO 源时间戳仍在检测日志保留，控制日志记录 source_odom_stamp_ns。

应用层的源时间年龄门槛改为接收侧检查。里程计/前置段 Sport 输入中断超过 1 秒仍请求停车，原有急停、控制权、节点退出、目标生命周期和任务时限继续生效。Point-LIO 与 IMU 驱动继续使用各自采样时间进行融合。使用不同采样时刻的最新数据会引入运动中的时空配对误差，现场定位、停靠和抓取效果 `NOT VERIFIED`。

部署备份位于 NX `~/go2_full_nx/releases/599c86c/latest_data_backup.56cbqaq0`。11 个源码/脚本变更及对应安装模块均已核对哈希，入口 SHA-256 为 `701fb8e87138ec875b68b3f03e749db5749334c6b6adf80c5677ca70e42fa031`。NX 实际安装模块读回 latest mode=true，4 项隔离 DDS 专项测试通过，60 项导航/历史到位回归通过。专项覆盖过旧、重复、未来、零源时间戳、最新 TF、最新深度选择以及数据停止接收后的停车。

本机大范围功能测试为 360 passed、16 failed、8 skipped，修改前基线同样有这 16 项失败，集中在旧 mock 和横向精停配置预期。并行 flake8 测试停滞后已清理，lint 未标通过。源码补丁与清单见 [最新数据补丁](go2_nx_latest_data.patch) 和 [哈希清单](go2_nx_latest_data_manifest.json)。本轮只运行隔离测试与离线计划，实机运动 `NOT VERIFIED`。

## 两轮识别与导航退出

只读对照 `full_1783582161085742040` 和 `full_1783582472852974808`。前一轮录像元数据有 93 份检测，其中 90 份含方块、77 个目标有有效深度、76 份发布 MAP 观测，目标 ID 2 有 CONFIRMED 记录。协调器进入 EXPLORE 后连续 5.003 秒未取得 `verified_vehicle_pose`，报 `Shared map TF or body odometry did not recover`，随后停止出口才触发控制器的 external stop。控制日志中 88 份定位就绪，但 89 份控制状态均没有共享的已验证位姿。

后一轮 FULL 录像期间 173 份检测全部无方块，171 份选择原因 no_color_block，正式目标列表始终为空。早期前置段曾有候选 ID 1，随后已被移除。探索阶段两次短暂定位等待分别约 0.372 和 0.387 秒；最后一次里程计样本年龄约 0.434 秒，超过 0.4 秒窗口，控制器报 `localization/TF: no fresh timestamp-matched odometry after arming`。此时 Sport 反馈年龄仍约 0.045 秒。Point-LIO 的 exit -6 均在收尾 SIGINT 之后出现，不能据此认定它先崩溃造成导航退出。第二轮定位样本为何未及时匹配仍待专项验证。

目标任务优先于探索，但需符合目标条件且定位可用。源码 `MissionManager.step()` 首先检查定位，再进入目标接近或探索分支。最新一轮录像期间没有方块目标可切换。

另确认了历史到位参数的副作用。原 `require_current_arrival` 同时控制共享位姿发布，关闭后协调器收不到 `verified_vehicle_pose`。本轮将位姿与 TF 一致性检查、已验证位姿发布和到位策略解耦，NX 源码与安装模块均同步到 SHA-256 `62c40e5cbc5970fbec8594f5f7712267d4a8f91d607bd02a0c9d7685e7a38b60`，备份在 `shared_pose_backup.gkm0zedr`。历史到位参数仍为 false。NX 隔离 DDS 的当前/历史两种模式晚到 TF 匹配测试 2 项、共享位姿消费测试 3 项通过。修复后的现场导航效果 `NOT VERIFIED`。

## 前置段到 FULL 的交接耗时

只读复核运行 `full_1783582472852974808`。该轮前置段持续 3 秒，DRIVE 时间戳为 1783582542.550，SETTLE 为 1783582545.560，DONE 为 1783582546.841。以计时直行结束为基准，停稳用了约 1.28 秒，FULL 协调器创建于约 7.61 秒后，进入 EXPLORE 于约 15.16 秒后，安全门首条非零 Move 于约 19.50 秒后，首条带平移速度的日志于约 28.55 秒后。Move 日志限频，这些指令时间为记录可见时间。

串行耗时包含以下步骤。前置段确认停稳后还等 Sport lease 到期，源码为最近一次续租期限加 0.25 秒，本轮 lease 期限为 1 秒。外层 `post_confirmation_readiness` 又占 3.135 秒，其中独立 ROS 探针占 3.019 秒。随后 FULL 加载协调器、重新启动安全门、地形与局部规划、接近控制器及录像器，协调器创建到 `CONTROL_STARTUP_READY` 约 7.23 秒，其中要求录像至少 10 帧、定位新鲜、导航端点就绪并连续稳定 1 秒。进入探索后到首条非零 Move 另有约 4.34 秒，期间发生一次 0.37 秒定位等待并生成探索目标。

因此延迟包含新增交接复核及原 FULL 自身启动、规划等待。原 FULL 发出的首批非零指令为转向，后续才出现平移。该轮随后出现 `localization/TF: no fresh timestamp-matched odometry after arming`，属于启动后的运行故障。证据位于本轮 `startup_timing.json`、`timed_straight.log`、`handoff_1783582553171557336.jsonl` 和 `1783582553171557336_gate.log`。本轮仅诊断，启动逻辑未修改。

## FULL 抓取位采用历史到位记录

按用户提供的第二种方案，FULL 启动参数 `dock_require_current_arrival:=false`。前后与横向各自达到原有容差即保存本轮到位记录。两份记录齐全且当前航向在容差内后，发送零速并等待原反馈停稳，随后走原有控制权释放、趴下和 S1 抓取流程。历史到位记录允许后续位置发生漂移，因此最终抓取位置须现场独立验收。

本轮使用原有历史记录逻辑，未加入 8 秒定时趴下。只有一个方向到位时继续控制剩余方向。故障、复位、反馈过期仍清除或阻断到位结果。更改位于 NX `source/camera_ws/scripts/run_map_anchor_experiment.py:297`，源码脚本直接被任务入口加载。备份为 `~/go2_full_nx/releases/599c86c/historical_arrival_backup.wdm9ip00/run_map_anchor_experiment.py`，部署后 SHA-256 为 `feed6c2cded2f659f703f6df8518af2394ee4cd64f7c915883329c369a13ada5`。补丁存于 [历史到位参数补丁](go2_nx_historical_arrival.patch)。12 项离线规则测试通过，实际真机行为 `NOT VERIFIED`。

## SSH 输入确认后拔 PC 网线

PC 上运行。

```bash
ssh -t -i /home/czx666/.ssh/id_ed25519_go2 unitree@192.168.123.18 \
  'python3 ~/go2_full_nx/go2_persistent_terminal.py'
```

此入口默认在 NX 持久终端中执行 `bash ~/go2_full_nx/go2_nx.sh --execute --mode full --1`。看到 `NX_FULL_READY` 后输入 `1` 回车，可拔 PC 侧网线。NX 独立持有终端、子进程和日志，SSH 断开后继续计时段与 FULL。NX 与 GO2 本体的连线需保持。

重新接入网络后，以下命令只连接已有任务。

```bash
ssh -t -i /home/czx666/.ssh/id_ed25519_go2 unitree@192.168.123.18 \
  'python3 ~/go2_full_nx/go2_persistent_terminal.py --attach-only'
```

连接后 `Ctrl+C` 请求原流程停止。日志在 NX `~/go2_full_nx/persistent_terminal/output.log`。后台任务结束后，默认启动命令可创建下一轮任务。已有任务运行期间默认命令重新连接该任务。

本机和 NX 均使用模拟程序验证终端为 TTY、输入确认、断开客户端后任务推进、重新连接及 Ctrl+C 退出。实机断 PC 网线后的 FULL 运行 `NOT VERIFIED`。

NX 已按用户要求移除计时段的雷达净空与地面覆盖条件。计时段按 IMU 航向循环发送 Sport Move，目标速度 1.00 m/s，起步渐加速，计时结束后发送零速并确认停稳。计时段启动和运行均无需点云。外层 FULL 的原有相机、定位和 S1 准备仍由原入口执行。

45 项本地离线检查通过，PC 和 NX 的 loopback 隔离 DDS 在完全不发布点云时通过计时完成、急停、Sport 反馈丢失和确认通道关闭四个场景。部署包位于本机 `/tmp/nx_timed_move_only/deploy.tar`。真机行走与 FULL 交接 `NOT VERIFIED`。

NX 入口读回 SHA-256 为 `76d395649605ceb92b709f0f1c4489df9af75b00a4e2009ee6a4d23195832ea8`，计时段加载 `source/scripts/nx_timed_move_only/go2_reactive_avoid`。旧入口备份位于 `~/go2_full_nx/releases/599c86c/timed_move_backup.J3BNIy/go2_full_test.py`。NX 离线计划读回通过。

下面的净空与覆盖说明记录此前部署版本的行为。用户后续说明现场没有纸箱，下面名为纸箱对照的样本应视为现场状态未确认，不能作为障碍物验收依据。

在具备原有运动条件且可使用物理急停时，从 **NX 本机终端**运行。

```bash
bash ~/go2_full_nx/go2_nx.sh --execute --mode full
```

1. 等待 `NX_FULL_READY`。原有预检会启动或复用相机、定位和检测链，并等待 S1 ZERO_HOLD。S1 服务缺失时，此阶段沿用原流程受控回零。
2. 拔掉 **PC 侧网线**，保留 NX `eth0` 与 GO2 本体的连线。通过这条网线接入的 PC SSH 会随之断开，因此请使用 NX 本机终端输入。
3. 键入 `1` 并回车。实时复核通过后，`NX_FULL_START` 表示开始启动自主任务。等待时可用 Ctrl+C 取消。

确认前，入口持续检查本轮启动的相机和定位进程是否仍在运行。确认后重新采样原生雷达、RGB-D、定位、TF、检测，复核 S1 ZERO_HOLD 和已有控制任务。检查失败时入口结束本轮。非交互式 FULL 调用在初始化前退出。

## 可选计时直行

```bash
bash ~/go2_full_nx/go2_nx.sh --execute --mode full --1
```

`--1` 设定直行 1 秒，`--2.5` 设定 2.5 秒。可用 `--timed-straight SECONDS`，范围为大于 0 至 10 秒。省略 `--execute` 可查看离线计划。

带计时段时，NX 在 `NX_FULL_READY` 前核对机载 Sport 端点、取得 Sport lease，并等待站立静止、IMU 和前向雷达净空。拔掉 PC 侧网线后，在 NX 本机终端键入 `1` 并回车。以确认时刻为基准延时 1 秒，锁定起步时的 IMU 航向，按单调时钟计时。前置段巡航范围为 0.80–1.00 m/s，从起步逐渐加速。到时发零速，连续确认停稳并等 Sport lease 到期后打印 `NX_TIMED_DONE`，随后复核 FULL 的传感器链并启动任务。

制动余量模型从机身前缘计，包含 0.55 m 余量、0.40 s 反应预算及 0.50 m/s² 减速度假设。对应 0.80 m/s 需约 1.51 m 已观测净空，1.00 m/s 需约 1.95 m。雷达只认可在行驶走廊内连续观测到的地面范围，障碍或覆盖中断会收缩可用净空。净空不足以支持巡航下限或当前速度时，前置段中止并请求停车。实际制动距离待实测。

整个过程中可用 Ctrl+C 和原有急停。净空不足、反馈超时、航向偏差超限或控制权异常会中止本轮，FULL 主任务不会接管。预检需要 GO2 已在平地站立静止，启动入口沿用原 S1 准备行为。直行时长由参数给定，实际位移受净空与加速限制。

### 计时段部署与验证

- NX 入口 SHA-256 为 `cb032e874c44fd7aae6b2d62dc2724a196e69558c4d64bb619d4b1d39c6dad88`。0.30 m/s 版入口备份于 `~/go2_full_nx/releases/599c86c/timed_speed_backup.q7LsO6/go2_full_test.py`。仓库保存[入口补丁](go2_nx_timed_straight.patch)，运动控制复用 [UTLiDAR 避障代码](../avoid_ws/src/go2_reactive_avoid/go2_reactive_avoid/) 和 NX 的 Sport 安全门。
- 本机离线测试共 45 项通过。NX Foxy 在 loopback 的隔离 DDS 域通过正常计时、急停、反馈中断、障碍出现和确认通道关闭五个场景。正常场景的最高发布指令超过 0.98 m/s，保持在 1.00 m/s 上限内。
- NX 真机只读端点审计确认 9 个 GO2 本体 Sport 发布者、零外部控制请求。实际传感器观察收到 201 帧 Sport 状态，雷达净空读取因地面回波不足未通过。另采 20 帧 Sport 时机身高度反馈约 0.074 米，未达到站立条件。此次无真机运动，站立时净空及实际按时停车和 FULL 交接仍为 `NOT VERIFIED`。

### 站立空旷和纸箱对照

- NX 本机日志 `full_1783581727118022040` 记录相机、定位、检测、S1 均通过，计时段因机身前缘 1–3 mm 的障碍候选和零连续净空退出。前一轮 `full_1783581645137221784` 在只读端点复核时收到 280 条外部 Sport 请求，属于另一种启动阻断。
- 现场确认站立空旷后，NX 于本机时间 15:29:42 采到 119 帧静止点云和 163 帧 Sport，机身高度反馈约 0.328 m。119 帧全部重现近距拦截，最近候选距前缘中位 0.20 mm。前缘 0.20–1.00 m 的三个前向条带累计没有进入地面高度判据的点。转角和安装偏置使用当前 NX 源码。
- 现场放好纸箱后于本机时间 15:31:37 采到 118 帧静止点云。前缘 0.75–1.10 m、横向 ±0.50 m 的范围，空旷和纸箱组在过滤前各仅有一个点，均落在地面高度带内。近距回波仍存在。现有点云流程未验证能可靠判定该范围的低障碍，近距回波的物理来源仍待定位。
- 原始数组和逐帧汇总存于本机 `camera_ws/records/nx_timed_clearance_20260709/`。该目录内保留空旷和放箱两组独立样本。此次只订阅原生雷达和 Sport 状态。
- 原 FULL 远距阶段使用处理后的地形图与局部规划器，NX 源码中规划限速为 0.4 m/s。视觉停靠另用目标反馈及自己的速度和停止条件。`--1` 以原生点云独立验证初始航向走廊和制动余量，不使用目标观测代替走廊覆盖。
- 本地修正分别报告近距候选、机身边缘回波和连续地面覆盖，明确预检失败时在进入 Sport lease 前退出。另修正确认时间在读管道前后发生的竞态。46 项离线测试和隔离 DDS 的 6 个场景通过。NX 连续关闭 SSH 握手，待部署版本 SHA-256 为 `9def471e32580d6cd2250154176533610bd40361998e3ca5b1f4ae558c9252e9`。

## NX 部署与证据

- 生效入口转到 `/home/unitree/go2_full_nx/releases/599c86c/source/scripts/go2_full_test.py`。部署前备份为同一发布目录下的 `startup_confirmation_backup.zgivme/go2_full_test.py`。仓库保存[该版本的补丁](go2_nx_full_startup.patch)，其适用基线 SHA-256 为 `eece32f6c412f584f36704a64d0c64030c2f29ed8ecc7cd719b90849e8b41244`，部署后为 `f3a2b232506cf7e3bc22ccee194d03fceeb7b98da105c28c0e336089e29075bd`。
- NX 本机计时比较使用同一组 12 个包及同一 ROS Foxy 环境。重复调用 `ros2 pkg prefix` 用时 5.228 秒，单次 Python 查询 0.056 秒，所有前缀匹配。原有 CUDA 检查和传感器就绪阈值继续执行。
- 离线终端和编排测试通过。覆盖等待期间的提前输入、错误输入、Ctrl+C、EOF、子进程退出及确认后就绪失效。NX 离线计划和 `--startup-check` 已运行。只读检查记录在 NX `camera_ws/records/full_1783581878305000440`，收到 45 帧原生点云，约 14.77 Hz，当时无 S1 服务及运行中的图像和定位流。
- 部署后 NX `--nx-sensors` 返回 `ready=true`，确认原生雷达、连续 RGB-D、Point-LIO、机身及相机 TF、YOLO 均通过。环境准备 11.04 秒，最终实时就绪 19.24 秒。记录在 NX `camera_ws/records/full_1783581894453287672`，退出后核对相机、定位和控制进程均已退出。
- 历史 `nx_sensor_yolo_cable_disconnected.log` 有断 PC 网线后的传感器链验证。新版确认点前后实际拔线、S1 启动和 FULL 真机任务仍为 `NOT VERIFIED`。
