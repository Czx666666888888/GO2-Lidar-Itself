# Project Status

## 2026-11-20 — GitHub 代码与主页同步

整理当前开发分支的既有源码、配置、测试、部署补丁和工程文档，更新根 README 的 PC/NX 入口、
独立导航、最小规则避障及 YOLO V3 状态；原始分析数据、视频、权重和构建产物留在本机。
用户已明确选择以 `feature/science-targets-perception` 为 GitHub 默认分支。
本轮离线复核：避障入口 47 项和导航/目标桥接/设备清点 39 项通过（后者有 2 条 NumPy 弃用警告）；
Python AST、shell 语法、README 本地链接及凭据特征检查通过。
NX 专项脚本直接在 PC 合并运行时缺少 `NX_NEAR_PREPARED` / `NX_SPEED_PREPARED` 部署快照，
收集环境补齐后仍有 22 项失败，不能把 PC 避障控制器作为 NX 变体验收；NX 历史通过记录未重跑。
本轮不连接机器人、不执行运动；真机效果 `NOT VERIFIED`。远端推送与默认分支结果以本轮终端读回为准。

## 2026-11-20 — PC 最小规则避障

按本轮用户要求，`avoid_ws` 改为固定检测框的 0/1 条件动作：默认 0.20 m/s 直走；
前缘 25 cm 触发停车后右优先 90°，左右同堵分两段右转掉头；侧边 15 cm 单侧触发反侧连续 20°弧线。
取消地面/方向覆盖、净空、旋转扫掠及连续距离调速；保留数据失效停车、Sport gate/lease、急停和时限。
`0` 仅代表未收到框内障碍回波，盲区和转身碰撞未被这些规则保护。
PC 唯一入口继续为 `bash go2_test.sh --mode avoid`，默认离线；操作详见 [avoid_ws/README.md](avoid_ws/README.md)。
来源：本轮源码、ROS 包构建及根入口 47 项离线测试；未执行真机，物理效果 `NOT VERIFIED`。
本节取代下方历史避障条目的旧速度、覆盖及扫掠规则。

## NX 靠近目标被遗留导航回调中断

最新 `full_1783581759499810552` 已识别目标 7、左侧并完成停车确认，导航进程已退出；约 5.5 秒接近后协调器先报 `Map odometry discontinuity`，原控制器随后才收到 external stop。上轮遗漏了导航 odom 回调和位于接近分支之前的导航 error 处理，冻结基准可能把累计位移当跳变。已在 NX 隔离这些回调，并从原控制器记录接近路段以保留多目标/返航轨迹。源码/安装模块一致，178 项离线回归通过；未启动真机，修复效果 `NOT VERIFIED`。见 [证据与补丁](docs/go2_nx_near_odometry_callback.md)。

## NX 前置段可调速度与最终可拔命令

NX 已支持整数后缀表示秒数、小数后缀表示速度：`--3 --1.0` 为 3 秒、1.0 m/s；小数时长用 `--timed-straight 2.5`。速度范围 0.10–1.00 m/s，默认 1.0，实际传至计时控制器和 Sport gate 上限。确认转交后新增 `NX_START_ACCEPTED` 提示。NX 正式源码 25 项离线检查通过，两个正式入口离线计划通过；仅回环模拟验证 0.6/1.0 m/s Move 上限、到时停止及急停。两台电脑完整命令和边界见 [最终命令](docs/go2_nx_final_commands.md)。本轮未执行真机或实际拔线，`NOT VERIFIED`。

## NX 接近时停止导航，保留多目标与侧一致性

按用户确认的原 PC FULL 接近控制边界，NX 现在在停稳交接后结束本轮 NBV/FAR/局部规划及其地形分析进程组，确认规划发布者退出后启动原 STOP_CONFIRM/接近。接近时不再受导航定位、地形、路径和规划端点检查限制；保留原接近定位检查、多目标循环、已完成过滤、目标 ID 和抓取侧一致性。唯一速度出口和 Sport gate 保留。NX 源码/安装模块读回一致，173 项离线回归通过；未启动真机。原定位连续性检查在旧 PC 代码中已存在，其现场异常是否消失 `NOT VERIFIED`。见 [部署与对照](docs/go2_nx_original_near.md)。

## NX 最近五轮未抓取只读复核

最新接收模式五轮均有方块检测记录；两轮定位连续性失败，一轮停车确认不稳，一轮近场交接一致性失败。另有一轮完成停靠并进入 S1，但腕部观测端点误差超出有界修正范围，返回 MISSED，其第二目标在停车确认阶段丢失。最新轮里程计接收年龄约 68 ms，不能归因为此前 400 ms 源时间戳门槛。具体运行 ID、证据和未知项见 [五轮诊断](docs/go2_nx_no_grasp_latest_runs.md)。本轮未修改控制、未发送运动；修复和重试 `NOT VERIFIED`。

## NX FULL 改用最新数据

已部署 NX FULL 最新接收数据模式。应用层取消源时间戳 400 ms 年龄门槛和同时间戳 TF/RGB-D 配对，改用最新 TF、最新接收里程计与图像。接收中断停车、急停及控制权检查继续生效。NX 实际安装模块的 4 项隔离测试和 60 项导航/到位回归通过。大范围功能测试的 16 项失败在修改前基线中相同。备份与验证边界见 [最新数据模式](docs/go2_nx_full_startup.md)。运动中时空配对误差及真机效果 `NOT VERIFIED`。

## NX 两轮识别与导航故障

两轮 `full_1783582161085742040` / `full_1783582472852974808` 分别有 90/0 份含方块的检测。前轮共享已验证位姿缺失 5 秒退出，后轮时间戳匹配里程计年龄约 0.434 秒触发控制器 FAULT。已修复历史到位开关同时关闭共享位姿发布的关联，NX 源码/安装文件读回一致，5 项隔离/离线检查通过。第二轮数据匹配中断的底层原因仍待验证。记录见 [两轮识别与退出](docs/go2_nx_full_startup.md)。

## NX 前置段到 FULL 耗时复核

只读复核 `full_1783582472852974808`，3 秒直行结束后约 15.16 秒进入 FULL EXPLORE，约 19.50 秒出现首条非零 Move 日志，约 28.55 秒出现首条平移 Move 日志。包含停稳 1.28 秒、lease 释放等待、串行实时复核 3.135 秒、FULL 节点启动与录像/定位稳定等待约 7.23 秒，以及探索规划等待。详见 [NX 交接记录](docs/go2_nx_full_startup.md)。本轮只读诊断，逻辑未改动。

## NX FULL 恢复历史两轴到位记录

按用户提供的第二种方案，NX `camera_ws/scripts/run_map_anchor_experiment.py` 已将 FULL 的 `dock_require_current_arrival` 改为 false。前后、横向分别记录到位，记录齐全且当前航向合格后归零并等待反馈停稳，再走原趴下交接。到位记录不会因同一轮后续位置漂移而清除。采用历史记录规则，无新增固定 8 秒趴下定时。原始脚本备份在 NX `historical_arrival_backup.wdm9ip00`，更新哈希为 `feed6c2cded2f659f703f6df8518af2394ee4cd64f7c915883329c369a13ada5`。真机停车和趴下位置 `NOT VERIFIED`。

## NX 输入确认后断开 PC 网线

已部署 `~/go2_full_nx/go2_persistent_terminal.py`。PC 通过 `ssh -t` 连接持久终端，在 `NX_FULL_READY` 后输入 `1` 回车，可断开 PC 网线。默认任务为原 FULL `--1`，NX 保留任务终端和输出日志，可用 `--attach-only` 重连。本机和 NX 模拟程序均通过确认、断线继续、重连、Ctrl+C 测试。实机断线自主运行 `NOT VERIFIED`。

## NX 计时段移除雷达门槛（已同步 NX）

按用户要求，NX 计时前置段已移除连续地面覆盖、前向净空和雷达数据就绪条件。前置段持续重复发送 Sport Move，IMU 保持起步航向，目标速度 1.00 m/s，时间结束后发零速并确认停稳。保留原有确认、急停、反馈超时、站立与 Sport 控制权检查。45 项离线测试通过。NX 无点云隔离 DDS 的计时、急停、反馈中断、确认通道关闭四个场景通过。入口读回哈希为 `76d395649605ceb92b709f0f1c4489df9af75b00a4e2009ee6a4d23195832ea8`，加载 `scripts/nx_timed_move_only`。真机效果 `NOT VERIFIED`。

## NX 本机自主 FULL 启动

NX 的 `~/go2_full_nx/go2_nx.sh` 已更新到人工确认流程。现有传感器链和 S1 ZERO_HOLD 就绪后打印 `NX_FULL_READY`，可拔 PC 侧网线，并在 NX 本机终端输入 `1` 回车。入口重新核验实时数据和 S1 后启动任务。NX 上 12 个 ROS 包前缀的查询实测为 5.228 秒降至 0.056 秒。改动、现场操作和验证边界见 [NX FULL 启动记录](docs/go2_nx_full_startup.md)。

NX FULL 支持可选 `--1` 计时直行。确认后等待 1 秒，以 IMU 保持初始航向，巡航范围 0.80–1.00 m/s。计时结束后停稳并交还 Sport 控制权，再进入 FULL。已部署并通过 NX Foxy 的隔离 DDS 五场景验证。上次真机只读时机身高度反馈约 0.074 米且无有效净空判定，站立后前置段、实际制动及 FULL 交接 `NOT VERIFIED`。

补采站立空旷 119 帧和纸箱对照 118 帧。空旷时全部复现近距候选，前缘约 0.8–1.0 m 的纸箱在原始点云未得到可区分回波。连续覆盖缺口仍阻断计时直行。诊断修正和确认时间竞态修正通过本机 47 项离线及 6 项隔离测试，NX SSH 握手被远端关闭，更新未部署。 [站立采样与部署记录](docs/go2_nx_full_startup.md)

## PC 一键入口可选洞口段

`bash go2_test.sh --execute --50--30` 在 MAP + S1 full/dock 流程前，按 Sport 里程计沿起始航向前进 50 cm，停稳后顺时针原地转 30°，再次停稳后交接。洞口段已移除 UTLiDAR 订阅和净空判断，保留 Sport lease、端点核验、反馈检查和急停保护。起步前只读检查已有 S1 ZERO_HOLD 及旧自定义遥控服务。39 项离线用例通过，含无雷达起步和 Sport 反馈失效停车。洞口实际位移、旋转及真机交接 `NOT VERIFIED`。用法见《GO2实机测试命令手册》。

## GO2 雷达纯避障独立测试

启动 `run.jX07Qnht` 和 `run.AOHdX70X` 均在 DISARMED 阶段因 Sport 发布端点数量检查退出。只读 DDS 发现仍有 9 个 `/api/sport/request` 发布端点，CycloneDDS 参与者地址均为 GO2 本体 `192.168.123.161`，6 秒订阅窗收到 0 条 Sport 请求。避障入口已加载本机核实的 9 个端点 GID，并保留其他端点、其他 Sport 请求和重复 lease 发布者阻断。24 项离线测试与隔离 DDS 测试通过。修复后的真机复核受 `enp12s0 NO-CARRIER` 阻断，无真机运动验证。

新增 `avoid_ws`，统一入口为 `bash go2_test.sh --mode avoid`。源码使用原生 UTLiDAR 四向障碍判断、Sport 航向和停稳反馈，支持右转优先的 90°闭环、两段 90°掉头、连续 20°弧线和停车前减速。前进指令 0.20–0.40 m/s，机身前缘距障碍 0.95 m 开始减速、0.55 m 停车。原地转向上限 0.60 rad/s，弧线转向上限 0.50 rad/s。ROS 包构建通过。历史包 retest9 有效雷达 1,732 帧，其中 360 帧满足旋转条件、667 帧后向覆盖未知。真机连续点云、旋转覆盖、Sport lease 和物理动作 `NOT VERIFIED`。操作见 `avoid_ws/README.md`。

## 2026-11-17 17:51/17:53未抓紧只读复核

用户指定full_1794909011145148930和full_1794909167286798775。17:51预抓取工具方向超差中止并回零，按匹配v4源码位于低位接触/正式夹紧前；非已证实力矩不足。17:53新任务accepted后至17:55:51仍GRASPING，末段约29秒反馈姿态不变，无新完成结果；last_log/result是前次残留，不能套用旧错误。NX多次SSH被远端关闭，原始姿态误差/第二次阶段/夹爪力矩未取得，具体卡点NOT VERIFIED。仅只读，无动作；详见docs/go2_arm_not_secure_20261117_1751_1753.md。

## 2026-11-17 第二轮距离仍远即趴下（离线诊断）

最新full_1794902087861996506/8c35804第二轮沿89秒前目标记忆接近，82份检测全空、无重新确认/锚点更新；一次估计7.937cm锁存到位，最终11.513cm且pose_in_tolerance=false仍在有效停稳0.500s后STOP并发StandDown。LIO窗口路径与Sport积分差30.853cm，localization_untrusted=true但差异停车关闭；不能把路径差视作真实终点偏差。腕部返回NO_TARGET_SAFE_RETURN，第二轮MISSED（总计2完成/1漏失），非三次抓取成功。物理偏差/定位底层原因未确认；无动作/算法修改，详见docs/go2_second_approach_20261117.md。 用户指出横向问题后分轴确认：锁存时前后-0.29cm、横向+7.93cm，最终横向+9.38cm；预测S1横向-41.49cm而目标-32cm。主要残差为横向，软件有正vy输出，真实横移量未独立验证。

## 2026-11-16 21:48指定录像有框未停车

full_1794836905838133718（7c17021/V3）双日志对照：指定MP4完整解码867帧；134份检测有有效深度，但正式MAP仅6次、源时间视频约35.146秒后中断，此后128份有效检测未成为正式点。控制未进入STOP_CONFIRM，SEARCH约53.4秒后定位跳变/时间回退FAULT。首次2.64m超1.8m触发距离；后续adapter拒绝reason未录，空间锁/TF/年龄具体原因仍待验证，不套用另一轮轮间旧锁结论。仅离线分析，见docs/go2_no_stop_20261116_2148.md。

## 2026-11-16 YOLO第二轮不停车确认

full_1794834675902500879第一轮起立后第二轮71帧方块深度有效，但正式MAP无新增、candidate/history全空。源码/安装一致：YOLO BlockSelection跨轮保持第一块initial/current，轮间仅恢复相机没有解除选择，阻断新目标进入上游正式候选。24.915秒SEARCH后另遇定位跳变FAULT。逐帧selection reason未录，诊断边界见docs/go2_yolo_round2_selection_20261116.md；无改代码/动作。

## 2026-11-16 20:38第二次左侧抓取卡住

第二轮约20:41:49起预抓取下降J6跟踪误差0.09105rad超过0.08，轨迹等待不推进，保持同一指令至少209秒；20:42:44随后CH341断连、重枚举ttyUSB1，旧进程仍持ttyUSB0。第二轮未回零/起立；第一轮left已软件完成。仅只读，无动作或算法修改。证据docs/go2_second_grasp_stuck_20261116_2038.md。

## 2026-11-16 18:50第一轮起立后第二轮FAULT（只读复核）

full_1794826213729797107记录第一轮S1回零成功、18:51:51起立反馈通过并计数1/3。相机恢复probe中RGB-D新鲜，但转换点云/IMU/odom/base约9秒未更新、slam_ok/TF/ready均false；恢复逻辑只验camera_ok仍启动第二轮，控制器robot=null并START_REJECTED，约2秒后FAULT/manual emergency stop。时序符合启动拒绝后协调器自动收尾/stop，发布者未独立记录，不能归因人为急停。待补全轮间定位就绪与追查数据中断，未改代码/动作；见docs/go2_post_grasp_fault_20261116_1850.md。

## 2026-11-16 18:45 STOP_CONFIRM只读分析

最新45e6b2f/HSV/full已通过预检并运行；前两次ID1/ID2超时，最后停稳后分别17/7份检测均无有效目标，复核样本0；第3次ID7在5.600秒确认成功并进入停靠。首次发现到锁定43.881秒。fast实际启用，但停车时Manager未确认会固定走完整回退；ID2虽具备可靠前史，停稳前即丢失/歧义。无算法修改/动作，详细证据docs/go2_stop_confirm_20261116_1845.md。与早先相机Depth0预检阻断区分。

## 2026-11-16 18:02无运动直接原因（只读）

最新45e6b2f在两次各30秒就绪采样后失败，Depth均0、RGB由132且陈旧变为0、检测0，机身/相机TF均false；点云/IMU/odom/base有新鲜数据。入口未启动SEARCH/安全门/抓取。18:05 NX仍枚举D435/D455，但D435设备号较17:47从6变8，表明期间重新枚举，具体重置/插拔时间及原因未明。PC无连续相机流已证实，USB物理掉线与NX采集/ROS/DDS具体故障环节仍待分段验证。未采流/动作/修改代码，见docs/go2_entry_diagnosis_20261116.md新增节。

## 2026-11-16 17:45一键入口最后就绪检查失败（最新只读复核）

最新00983d0/hsv/full在final_live_readiness等待30.950秒后失败，SLAM/body TF正常但没有有效Depth/检测；D435报RT IC2 Config error、硬件重置、UVC Protocol error及No such device。17:43轮为launch加载TypeError阻断，17:44轮就绪通过后SEARCH发生定位/多路数据超时，均须区分。当前PC入口已退出，校时已恢复；本轮仅只读，无动作/采流/重启。NX内核日志权限不足，具体底层根因待核验；证据docs/go2_entry_diagnosis_20261116.md新增节。

## 2026-11-16 PC查询超时直接原因已定位

journal证实16:29:40 PID1重新执行后在libsystemd-shared-249.so发生general protection fault，Caught SEGV并Freezing execution；D-Bus的systemd1未连接，timesyncd本身仍运行。自动更新同时替换libc6，时间相关已确认但精确崩溃触发链待core分析。另有新6.8.0-149内核NVIDIA模块构建失败/包未配置，当前仍运行6.8.0-138；恢复时需考虑。仅只读，未重启/修系统/运动。证据docs/pc_systemd_crash_20261116.md。

## 2026-11-16 用户授权跳过PC校时查询启动一次

用户明确确认路线空旷、急停/监护、S1初始位和支撑，并授权一次full --view，仅绕过PC校时服务查询。以GO2_PC_TIME_HELD=1调用原入口，未更改默认保护。实际5e818cf版本准备1.718秒、设备检查0.596秒、复用S1 ZERO_HOLD约0.190秒；existing探针0.715秒退出但未生成报告，入口退出1，未进入相机/定位启动、行走、趴下或抓取，不重试。探针提前退出原因Unknown / needs verification。原S1保持保留。证据camera_ws/records/forced_full_20261116_164327及full_1794818619293939823。本轮未暂停PC校时服务，真实物理结果NOT VERIFIED。

## 2026-11-16 PC校时包装器故障阻断修复

主入口在转交独立集成目录前使用本仓库pc_time_check.py，避免改动集成clone及其远端一致性校验。状态查询限3秒，仅明确active/inactive可继续；D-Bus报错/未知状态阻断，stop/start限10秒。初始查询纳入计时，异常和恢复失败均落盘；运行中查询失败先请求原任务退出，再恢复校时。15项离线故障注入与入口转交检查通过，未执行full/部署/重启。systemd根因和真机效果NOT VERIFIED。见docs/go2_pc_time_guard_fix_20261116.md。

## 2026-11-16 一键启动只读诊断

最新full准备约3秒、S1约5秒，因原生雷达约1.7Hz/报告loss约91.7而在相机/定位前阻断。外层systemctl查询曾等待20秒且无timeout；先前准备失败详情未持久化。另见包装器强制right覆盖full默认auto，实际默认自动选侧说法纠正。后续NX已刚重启、S1 socket缺失，不代表失败时状态。未部署/重启/动作，见docs/go2_entry_diagnosis_20261116.md；实机NOT VERIFIED。

## 2026-11-15 双相机并行复现USB异常

单D455直连/Hub三轮均出图，单D435 Hub短测220帧无断连；同Hub双相机同时采流，
双线程和两个独立进程均复现超时/USB断连。独立进程轮D4550帧、-71共9条、断连2次。
证据：camera_ws/analysis/dual_camera_comparison_20261115.json及d455_usb_diagnosis_20261115.md。
采集已释放，S1保持USB1 ZERO_HOLD，无运动；后续分开物理端口验证，供电/并发根因及实机抓取NOT VERIFIED。

## 2026-11-15 D455掉线代码与官方资料核查

- 只读核对NX原始process_result、实际SDK动态库/固件、USB省电与内核日志：SDK2.54.2+FW5.17.0.10，官方表该FW对应2.56.5；为待验证版本差异，不能直接断言不兼容。
- 20:00复拍在query_devices枚举全部相机时失败，尚未选中D455；20:12在预热取帧超时，D455多次实际断连重枚举。代码有finally stop及worker退出等待，未发现正常路径双采图并行。
- 证据/官方链接与下一步隔离验证：camera_ws/analysis/d455_usb_diagnosis_20261115.md。本轮无采流/动作/SDK或省电修改，具体底层根因与修复效果NOT VERIFIED。

## 2026-11-15 19:29 最新full失败：原D455初次采图5秒超时

- 只读证据full_1794742134203337027（9b7f281、HSV）：曾进入71.107mm并锁存到位，最终88.907mm/合格停稳0.6001s后STOP；此为用户要求的历史到位规则生效。
- 19:29:45.845 StandDown RPC返回0，19:29:48.816姿态反馈通过并接受抓取请求。原抓取流程initial_capture_detection报`Frame didn't arrive within 5000`后中止，不是已删除的D455录像前置检查。
- 本轮相同NX monotonic89～137窗口有D455 usb2-2.3 UVC -32/-71、多次断连重枚举；直接失败为无图像，USB链路异常已观测，线材/供电/驱动具体根因未确认。
- 末次S1日志为ZERO_HOLD、busy=false、at_zero=true、通信OK、last_exit_code=1；原协调器拒绝自动起立。本轮未重试或动作，物理状态独立确认仍NOT VERIFIED。

## 2026-11-15 18:55 趴下位置错位：HSV锁中右下角近处区域

- 用户确认现场为同一物块后，本轮将full_1794740089097906377的原始/叠加视频逐帧解码，并把实际日志contours/center回画到源RGB帧。结果纠正此前“66cm一定为同一物块MAP漂移”的表述：自动检测主要锁中画面右下角近处青色区域，非画面中可见的独立方块。
- 85份检测、87个接受候选中85个在右下角，u574.60～585.35/v427.71～438.27，深度0.366～0.480m；仅2个候选在别处。初始STOP_CONFIRM/锁定由右下角观测建立；场景同一物块不等于检测器始终识别该物块。
- 对该近处区域建立的旧MAP锚点为(0.379011,-0.244054)，移动后新观测(1.033874,-0.152131)相差0.661283m，被initial_anchor_radius拒绝；控制仍对旧锚点完成STOP并发起趴下。因此2.66cm停车误差不是相对真实方块的误差。
- HSV条件H90～135，只有颜色/面积/深度/蓝墙剔除，无相机自身遮挡区域排除。日志wall_detected=0，未剔除该近处区域。压缩RGB离线重放显示真实方块色相在下限附近、后期部分连通面积低于250；初期为何未进入深度候选因无原始Depth不能精确复原。
- 证据：camera_ws/analysis/target_anchor_20261115_1855/内summary.json、detections.csv、logged_detection_evidence.jpg、analyze.py。原始与叠加160/160帧完整解码、JSONL对应、源文件前后SHA256一致。未改生产算法/阈值、未重启/运动。

## 2026-11-15 18:28 HSV full：趴下通过后D455录像前置失败

- 只读核对最新 `full_1794738466158509218`，版本049feb1、detector=hsv。导航最终STOP/complete=true，停车位误差67.327mm、合格停稳累计0.5999s；18:28:29.198 StandDown返回0，18:28:32.264高度/模式/静稳反馈通过，非独立物理接触证明。
- 抓取请求accepted=true进入GRASPING，但新增grasp_video前置录制在首帧检查失败，`Frame didn't arrive within 1000`、frames=0，未调用原抓取运动回调；S1为FAULT_HOLD、busy=false、串口USB0通信OK，未自动起立。
- 对应NX monotonic1276～1279内核日志：D455 usb2-2.3出现UVC -32/-71、两次USB disconnect与重新枚举。直接阻断是录像前置无图像，USB底层异常已观测；不能仅归因1秒等待短，也未确定供电/线材/驱动具体根因。
- 前侧303帧closed，腕部元数据已取回但0帧/complete=false；COPIED不等于有效录像。本轮无动作/重试，真实抓取NOT VERIFIED。证据：该目录s1_task_20261115T102826Z.jsonl、recordings.json和run JSON；另为本轮SSH只读内核/S1状态输出。

## 2026-11-15 18:13 最新full FAULT只读复核

- 本轮日志证据：`camera_ws/records/full_1794737564166868040/`，实际版本65ff2b5。首次FAULT为18:13:24.582，停靠11.400s，原因`localization untrusted: sustained LIO/Sport motion disagreement`。
- 失视觉段LIO窗口路径0.765755m、Sport速度积分0.455870m，差0.309885m；代码每2秒审计，超过0.08m连续两次后锁存。不等于实际机器人走了该距离，也不能直接确定LIO漂移或Sport反馈哪方错误。
- 当时停车位误差0.123728m、视觉年龄6.562s、里程计年龄0.028s；曾最小误差0.050693m并进入STOP_WAIT但未完成。未进入趴下/抓取，D455录像NOT STARTED；前侧双录像记录321帧且closed，未在本轮重新解码。
- 全段Sport mode=0/error_code=100，语义及运动估计差异根因待查；启动lease超时随后成功，不是本轮直接FAULT触发；base_odom重复shutdown及gate类型转换异常位于停止收尾，另记问题。仅只读分析，无新运动或算法修改，物理结果NOT VERIFIED。

## 2026-11-15 单次 full --view 设备预检阻断

- 用户确认现场条件并授权 `bash go2_test.sh --execute --mode full --view` 一次；实际集成版本 ac86bf7，准备/构建阶段通过，设备清点因 ttyUSB0/ttyUSB1 均不存在退出1。
- 未进入S1使能、相机/定位启动、行走、趴下、抓取或起立；预览未启动，不自动重试。PC校时恢复退出0且 active=true。
- 事后只读SSH：无 ttyUSB/ttyACM/by-id，lsusb仅两个Linux root hub，未发现S1/抓取/RealSense进程；USB供电/接线根因待现场确认。NX时钟仍为2022-09-08。完整流程 NOT VERIFIED。
- 证据：`camera_ws/records/full_1794730495368390980/`、`pc_time_1794730492282287702/report.json`、`full_manual_20261115_1817/terminal.log`；目录标签不代表实际启动时刻，以日志为准。首次工具终端停在sudo验证后取消，未进入动作，随后桌面终端执行本次预检。

## 2026-11-14 服务自身串口自动识别

- 用户澄清“0/1都可以，不要反复更新”，集成提交`9b49644d9c774167bc1df7b5a69c4f92d921775a`补到原S1服务：默认auto、历史显式0/1也识别当前唯一端口；状态记录实际端口，启动器按打开FD检查。
- 保留唯一串口所有者，设备重建身份检查阻断旧SDK缓存；限定旧版本hash升级并备份，28项离线检查通过。
- 当前有线DOWN、SSH不可达，未部署/重启/动作。新版部署一次后，后续端口换号无需改配置；运行中USB断连仍须安全恢复连接，不能仅改字符串假装连接有效。实机NOT VERIFIED。

## 2026-11-14 前向响应检查与快速交接已提交

- 实际基线235b1dd，集成提交`9dd98a99888aa9822b7d8ee0c1f1bf1bf2046555`新增正向实测短步准备、同入口受控响应检查、事件驱动控制权释放和分阶段JSONL。131项针对性检查、单包构建及安装读回通过。
- 正向补偿需审核过的实测profile，当前默认未开启；不猜脉冲时间、不补偿后退/vy/wz、不改原制动参数及4cm/10°/0.5秒。既有低速499样本Sport接近零支持核验死区，非完整标定。
- 本轮无新运动授权，未运动/抓取。下一步同入口`--execute --mode dock --forward-response-check`在新授权和现场条件下测0.15前向响应/制动，再审核参数→独立dock→full。全部实机效果NOT VERIFIED。
- 具体手册和experiments已随集成提交推送；原主仓库脏改动保留。

## 2026-11-14 精调请求下限0.18m/s（当前）

- 按用户要求，集成提交`235b1dd25c037401404a51f4b3c204e0cafc213e`增加4～15cm精调请求下限0.18，并移除旧0.08上限，沿用0.30总上限；远段下限0.20保留。
- 原制动/实测速度/加减速优先，因此6cm附近仍可降至约0.089m/s；4cm零速等待与完成门槛不变。97项相关离线检查、源码lint、单包构建、安装代码/参数核对和离线计划通过。本轮未运动，实机效果NOT VERIFIED。

## 2026-11-14 17:37新版停靠门槛复核

- 最新340fd13/full日志最小停车位误差58.357mm，全程未进4cm，因此STOP_WAIT为0次；用户现场确认感觉已停稳，日志502个周期平移速度必要条件达标，不能归结为一直没停稳。
- 最终因失视觉剩余路径预算不足FAULT，未进入趴下；4cm仍为完成精度、6cm仅等待退出滞回。低速0.08m/s请求下Sport反馈近0的原因、真实停车参考与定位漂移需进一步核验。
- 证据与全部发起趴下门槛见`docs/go2_onekey_run_20261114.md`末节，本轮仅只读分析。

## 2026-11-14 停靠精调与零速等待已实现（未真机）

- 用户批准方案，集成提交`340fd139eb0031690902a474c52cb84be922d1ce`：15cm内精调请求≤0.08m/s，远段0.20m/s下限保留；4cm/10°进入零速等待，6cm外或yaw超限恢复修正，外圈等待2秒也恢复精调。
- 最终仍需原4cm/10°及多路速度/新鲜度连续0.5秒；5m失视觉预算、安全门、普通MAP/SEARCH及趴下/S1动作不改。
- 91项相关离线检查、单包构建、安装代码/参数读回和源码lint通过；本轮未启动运动。真实停稳效果与视觉长期未更新根因NOT VERIFIED。实现和后续独立dock验收见集成分支手册/ADR-013。

## 2026-11-14 最新17:26～17:28趴下阻断（日志复核）

- 最新full_1794648390211512595已通过预检并进入停靠。曾误差29.65mm但未持续停稳0.5s，最终FAULT且grasp_docking_complete=false，未进入StandDown；与前轮无深度启动失败区分。
- 首次FAULT时失视觉累计路径4.992079m，剩余预算7.921mm小于进入4cm容差所需14.714mm，触发remaining docking distance exceeds odometry budget。视觉年龄62.311s，长时未更新与反复修正根因待查。
- 详细证据及度量定义见`docs/go2_onekey_run_20261114.md`末节；本轮仅只读分析，无动作或算法变更。

## 2026-11-14 USB1保持后全流程启动失败（无运动）

- 用户再次要求全流程，版本951c8f7、0.20m/s下限，设备和S1保持通过；运动前30秒收到RGB428帧但对齐深度0帧，TF/就绪检查失败。D435日志连续Frames Timeout，底层根因待查。
- 原入口重建定位时排他创建已有slam.log报Errno17，退出1，未进入SEARCH/趴下/抓取。该日志名缺陷与无深度分别待修复/诊断，不自动重试。
- PC实验节点及远端相机已退出，校时active；USB1 S1保持继续且ZERO_HOLD。证据见`docs/go2_onekey_run_20261114.md`末节；完整流程NOT VERIFIED。

## 2026-11-14 S1串口0/1兼容

- 按用户要求，集成提交`951c8f7ba6cf0a6b3efd037bf37c7a3268664f26`允许唯一存在的ttyUSB0或ttyUSB1，传入原S1服务；18项离线检查、脚本及远端枚举语法检查通过。
- 双端口/缺失/旧服务绑定不一致时停止，保留原保持服务且不创建第二串口所有者。本轮未重启远端、未运动，真实通信恢复NOT VERIFIED。
- 后续用户另行授权安全重启：旧PID3107正常退出后，原服务PID17836在USB1使能保持；启动已在零位，无非零位回零运动。连续5次ZERO_HOLD/at_zero/通信OK，唯一串口所有者；保持继续运行。无行走/抓取，物理姿态待用户确认，证据见执行记录末节。

## 2026-11-14 停靠请求速度下限提高至0.20m/s

- 用户明确要求，实际集成分支提交`cb99af0889feb8b7b85aaeb8644a27d46fcd88cd`将`dock_min_linear_speed`由0.10提高至0.20；单包构建、安装值读回、37项离线检查通过。
- 制动/实测速度/加减速限制优先，最终命令可低于该值；搜索0.25、停靠上限0.30m/s及到位/趴下条件不变。此次未执行动作，实机效果NOT VERIFIED。
- 后续用户授权完整测试一次：cb99af0启动在设备清点因ttyUSB0缺失退出1，未进入动作；实读出现ttyUSB1，但原S1服务仍绑定ttyUSB0，未擅自重连或停保持。PC校时已恢复，串口身份/重枚举待核实，见`docs/go2_onekey_run_20261114.md`末节。

## 2026-11-14 一键入口恢复与单次执行

- 来源：本轮脚本检查、离线测试、单包构建和只读设备检查。
- `bash go2_test.sh --execute` 自动恢复缺失的 `/tmp/go2_onekey_s1`，校验分支及导航底包/记录目录链接，再转入远端集成流程；保留主仓库已有修改。
- 本轮实际集成提交 `ddbe7e6c9b233134ce4c63ab657dc54ff648d336`。构建通过，设备清点失败：伴随主机 `/dev/ttyUSB0` 不存在；两台指定相机可见，SSH/有线路由正常。
- 未到S1使能、搜索或抓取；PC校时服务恢复active。完整动作仍 `NOT VERIFIED`，不得将本次入口执行视为动作通过。
- 原始证据与下一步见 `docs/go2_onekey_run_20261114.md`。不自动重试运动，不修改串口绑定或控制保护。
- 同日另获授权重启后设备就绪并进入搜索/停靠，随后网线脱落（用户确认）、定位超时退出，未抓取；用户已急停确认停稳。按要求清理PC实验进程，校时恢复active，双录像各432帧完整；远端断网无法确认其进程退出，NOT VERIFIED。详见同一记录后节。

更新日期：2026-11-12
状态来源：当前仓库静态阅读、根 `README.md` 的既有记录和 Git 历史。除非特别说明，本文件不代表本轮重新完成了真机构建或运行验证。

## 一键实机测试入口（2026-11-12）

来源：当前源码、手册与实际fetch后的集成分支 `a8f7c667`，本轮离线输出。

- 根目录 `bash go2_test.sh --execute` 自动恢复独立集成目录、构建、网络/设备/旧进程检查、
  对时、S1保持、D435、SLAM/感知预检到原完整搜索/停靠/抓取/起立协调器。
- 默认只打印离线计划；运行时读取集成分支最新远端hash并记录，不覆盖主仓库脏改动。
  退出关闭本轮相机/定位，保留S1保持；不修改控制器、标定、保护阈值或失败恢复策略。
- 2026-11-12后续现场执行：修正自建链接误判、SSH握手计入对时RTT、后台保持启动
  未脱离SSH、原生雷达epoch被误判为PC时间等启动问题；9项离线测试、静态检查通过。
  单包构建成功；另一次colcon段错误139保留日志，底层原因未确认。
- 一键入口已实际完成连接/对时/设备检查、S1 ZERO_HOLD、相机/SLAM、连续转换数据
  预检并进入SEARCH、STOP_CONFIRM和三自由度停靠。停靠12.5s时机身里程计时间戳
  超前108.354ms触发原100ms保护，FAULT退出，未趴下/抓取/起立；用户确认已停住。
  最近MAP停车位误差46.15mm，未达40mm且未停稳，不构成停靠通过。原始/叠加录像各605帧。
- 最新入口复用同commit的已成功构建overlay，构建标记置于独立clone的.git元数据；
  主仓库/集成源码改动仍阻断。已停用遥控服务不重复请求sudo停止。未修改时间保护。
- 完整停靠/抓取端到端仍未通过：`NOT VERIFIED`。时间戳超前的底层原因需额外只读证据。
  依赖已部署导航底包、GO2相机/S1环境及原手册中的实际设备绑定。
- 说明与停止路径：`GO2实机测试命令手册.md` 首节；控制/录像仍写主仓库records。
  本轮记录：`run_1794494783627546512.json`、`full_1794494771455097289/`及实验文档末节。

## 科学目标生成器采样审查准备（2026-11-12）

来源：远端0014177、新源码、11项离线测试、Blender几何/失败路径检查；没有新RGB渲染。

- 距离改为near/medium/far候选及权重，0.4–0.8/0.8–1.5/1.5–2.5m；移除隐式尺寸放大。
- 新增七区域图像位置采样和部分出画；标签仍仅取画内Mask，新增12像素框短边门槛。
- 岩石复用Trimesh稳定姿态并检查坡面支撑多边形/重心投影，均匀密度仅为建模假设。
- 固定视角的距离段/区域不因失败切换；记录拒绝原因，无效样本不降门槛凑数。
- 保留三类、两个scale1.0、尺寸未确认和第六色null；CameraInfo/配置/工具代码hash可追溯。
- 2455个既有数据/池/模型文件hash和集合未变；未重新生成600张、未导正式池、未训练。
- 新采样RGB/Mask端到端与实物尺寸/重心/CameraInfo仍NOT VERIFIED；见合成GENERATOR_REVIEW.md。

## 科学目标合成数据准备（2026-11-12）

来源：远端起点 `91e11fec99cc2203eac40040049bdf9fe9f91e74`、原STL/发布ZIP逐字节核对、
Blender 4.2.1 + BlenderProc 2.8.0 CPU实际渲染及独立掩码/网格检查。

- 新增独立工具 `vision_training/synthetic_generation/`，不改训练核心或ROS2节点。
- 50mm基础立方体、0.5mm参数化倒角、五个已知色材质与第六色未确认占位已实建；
  第六色不采样，全部颜色仍为类0。原STL只读复制，比例分别配置，默认1.0。
- 目标1/2分别映射compact/spiky；单位未知，原坐标尺寸不等于已验证实物米制尺寸。
  拓扑封闭/绕序一致；目标2有8个近零面积小面但无零面积面，自相交未验证。
- 修正小批量发现的坡面接地符号错误与多目标间距问题；最终12张预览通过目视和
  mask→YOLO检查，18次原STL世界坐标接地独立检查通过；旧悬空样本被回归检查拒绝。
- 600张已实际生成并全量自动检查通过：主目标每类200，300场景，train480/val120；
  仅抽查最终预览，不声称全部逐图人工验收。证据见 `vision_training/synthetic_generation/REPORT.md`。
  按摆放场景划分train/val；合成数据只用于开发，不进入最终test；导入池保持未审核。
- 相机使用临时K并标记 `synthetic_camera`；实际D435 CameraInfo导入、Depth选项、
  GPU渲染、实物尺寸/材质匹配、真实D435泛化均 `NOT VERIFIED`。
- 未启动训练、相机、ROS或机器人；既有bridge/10mm未提交改动保留且排除本轮提交。

## 正式科学目标视觉准备（2026-11-11）

来源：开始前实际远端 `bfa7425`、本轮源代码/真实历史20图哈希核对与针对性测试。
当前开发分支 `feature/science-targets-perception`；下方原研发分支摘要为上一轮记录。

- 采用 RGB YOLO11n + D435 Aligned Depth，相机系类别/多框/粗点输出；仅视觉范围。
- 新增 science_targets_v1 三类：color_block / rock_compact / rock_spiky。
  20份旧blue_cube人工标签复制并映射为类0，原数据不改；同实验室组整体保留，新三类
  完整性待人工复查；没有岩石样本、正式split、三类权重或正式训练。
- 新增CVAT Detection导入、RGB-D近时间匹配采集与原值保存、训练预算检查及操作指南。
  34项针对性测试、静态lint、原V2配置解析通过；仅本机隔离域采集节点两秒启动退出、0帧。
- 本机未枚举D435，真实采集/时差/对齐与三类识别粗定位为 `NOT VERIFIED`。
  nbs4暂作batch4无累积起点，正式批次数与更新数须待真实划分后核对。
- 未修改HSV、深度阈值、MAP转换、导航/运动；原bridge/10mm未完成改动保留并排除提交。
- 操作入口：`vision_training/SCIENCE_TARGETS_GUIDE.md`；证据与限制：
  `vision_training/reports/science_targets_v1_setup.md`。未全仓构建、训练或启动GO2运动。

## 最新状态摘要（2026-11-11）

来源：当前分支 `feature/reusable-yolo-training`，实现基线 `5fa5cd0`、既有训练报告及
`camera_ws/docs/experiments.md`。下文早期基线与分阶段记录保留其日期和验证边界。

- 已有雷达导航链、独立 D435/HSV/map 目标管理、独立搜索接近与 YOLO 工程工作流。
- 5 m / 0.25 m/s 搜索、6 s 确认版本已取得单场景用户实测 9.5 cm 停稳证据；
  真实 5 m 路线终点、ODOM 边界、重复精度和正式相机外参仍待验收。
- blue_cube V2 已完成 80 轮：conf0.25、IoU≥0.5 时 last 的训练 TP/FP/FN=14/0/0、
  验证=4/0/2。20 图同场景 14/6，无 test；不证明泛化或部署可用。
- 本次仅更新根 README、相机 README、状态与任务说明及 GitHub 简介；未重新训练、
  构建、启动 ROS 或操作机器人。历史测试数字来自对应报告，不是本轮重跑。
- 本地遗留 `science_goal_bridge.py` 及其测试改动、`audit_10mm_initial_lock.py` 和
  `analysis/target_10mm_audit_20261108/` 为未完成工作的现状，本次保留且不纳入提交。

## Project Goal

本项目旨在使用 Unitree GO2 自带 UTLiDAR 和 IMU，通过 ROS 2/CycloneDDS 构建一条包含传感器变换、Point-LIO 定位与点云注册、地形分析、前沿探索、FAR 全局规划、局部路径规划、路径跟随和安全运动请求的自主导航链，并提供离线回放、分析、标定和分阶段验证工具。

## Current System

仓库中可以确认存在以下模块：

- GO2 DDS 接入与 Sport API 消息定义
- GO2 原生 UTLiDAR 点云和 IMU 输入预检
- `transform_sensors` 传感器坐标、时间戳和 IMU 数据处理
- `point_lio_unilidar` 定位、注册点云和里程计输出
- `terrain_analysis` 与 `terrain_analysis_ext`
- FAR planner、visibility graph 和 graph decoder
- local planner 与 path follower
- `wp5_explore_node` 前沿/waypoint 探索逻辑
- `base_odom_node` 雷达中心到机身中心的里程计转换
- `go2_safety_gate` 的 DISARMED/ARMED/FAULT 状态机、限速、超时和 Sport lease 处理
- 键盘遥控、传感器记录、RViz 插件、仿真/示例包
- build、预检、启动、停止、rosbag 回放、分析、标定和测试脚本

## Current Architecture

当前主脚本 `code/scripts/nav_launch.sh` 描述并启动的主链为：

```text
GO2 /utlidar/cloud + /utlidar/imu
  -> transform_sensors
  -> /utlidar/transformed_cloud + /utlidar/transformed_imu
  -> Point-LIO
  -> /registered_scan + /state_estimation
  -> terrain_analysis / terrain_analysis_ext
  -> /terrain_map + /terrain_map_ext
  -> wp5_explore_node
  -> /goal_point
  -> FAR planner
  -> /way_point
  -> localPlanner
  -> /path
  -> pathFollower
  -> /cmd_vel_raw
  -> go2_safety_gate
  -> /api/sport/request
  -> GO2
```

`base_odom_node` 将 `/state_estimation` 转为 `/base_state_estimation`，FAR launch 将其映射为自身的 `/odom_world` 输入。详细接口和不确定项见 `docs/architecture.md`。

完整 TF 在真实运行时是否连续且一致：`Unknown / needs verification`。

## Current Progress

### Confirmed implemented

以下仅表示源码、配置或脚本已存在并形成静态连接：

- 24 个 ROS 2 package 已进入仓库。
- GO2 原生 UTLiDAR 的 `/utlidar/cloud`、`/utlidar/imu` 预检脚本已存在。
- Point-LIO 的 UTLiDAR 配置与 launch 已存在。
- terrain、FAR、local planner、path follower 和 WP5 的主接口在源码中可定位。
- safety gate 默认 `dry_run=true`、初始状态 `DISARMED`，代码包含显式 `/arm`、`/stop`、心跳/命令超时、倾角检查与 Sport lease 逻辑。
- `deploy_local.sh` 可生成 `deploy_ws` 并执行 colcon build。
- 主启动与停止脚本、离线回放脚本和若干测试脚本已存在。

### P0 ROS 2 interface baseline (static)

- 已完成 24 个仓库内 ROS 2 package 的静态清单，并按主链、辅助、仿真和旧/替代流程分类。
- 已从 `nav_launch.sh` 及其 launch include 树确认主链 node、关键 topic、消息类型、参数来源和 remap；详见 `docs/interface_inventory.md`。
- 主数据流第 1 至 8 段均可由仓库源码静态闭合；safety gate 到 GO2 的第 9 段只能确认本地 publisher，GO2 外部 subscriber 和物理执行仍为 `NOT VERIFIED`。
- 静态 TF 意图为 `map -> camera_init -> aft_mapped -> sensor -> vehicle`，并有 `sensor -> camera`；Point-LIO 发布 `camera_init -> aft_mapped` 动态 TF。
- `transform_sensors` 内的 `body -> utlidar_*` TransformStamped 没有广播，不能作为实际 TF edge。
- 本轮没有启动 ROS graph，实际节点名、QoS 协商、TF 连通性与消息频率均为 `TODO: runtime verification required`。

### Present in code but not yet verified

- 当前 commit 在干净环境中的完整依赖安装与全量 build：`NOT VERIFIED`。
- 所有节点、topic 类型、QoS 和 TF 在同一次运行中的一致性：`NOT VERIFIED`。
- rosbag 离线全链复现：`NOT VERIFIED`（bag 未进入 Git）。
- GO2 实时 LiDAR/IMU 输入、Point-LIO 精度和地形输出：`NOT VERIFIED`。
- FAR 与 local planner 的闭环稳定性：`NOT VERIFIED`。
- safety gate 到真实 GO2 运动执行的端到端效果：`NOT VERIFIED`。
- 仓库内测试脚本在当前 baseline 的通过情况：`NOT VERIFIED`。

### Known incomplete / problematic

- 大量旧脚本和文档硬编码 `/home/lch/dog/...`，并非都适用于当前 checkout。
- 根 README 引用的 `today_representative.md` 不在仓库中。
- 5 组代表性 rosbag 仅在本地目录中，受 `.gitignore` 排除，ChatGPT 无法通过 GitHub读取原始数据。
- `transform_sensors` 从用户 Desktop 读取 `imu_calib_data.yaml`，文件缺失时使用源码默认值；当前真实标定来源和有效性尚未确认。
- 仓库包含旧流程、当前 DDS 流程、仿真和实验脚本；package 已完成静态分类，但 shell 脚本的正式支持/弃用清单仍未建立。
- Point-LIO 配置使用 `use_sim_time: true`，FAR launch 显式设置 `false`，其余主链节点通常使用 wall time；真实 launch 未启动 `/clock` publisher。
- `mapping_utlidar.launch` 将顶层 `use_imu_as_input` 设为 `false`，与 `utlidar.yaml` 中的 `true` 不一致。
- `go2_keyboard_teleop` 与 `transform_sensors` 的 manifest/setup 未声明源码实际使用的全部运行依赖，干净环境 rosdep 完整性待验证。
- Point-LIO 与 local planner 都声明 `/path` publisher；当前 Point-LIO YAML 通过 `path_en=false` 禁止其 path 数据输出。

## Known Experimental Results

以下内容仅转录自根 README，未在本轮重新分析 rosbag：

- `retest4`（78 s）：记录为能走 0.82 m，但 94% 时间在转。
- `retest5`（258 s）：记录为当天最远 2.96 m，仍有 67% 时间在转。
- `retest6`（219 s）：记录为走廊检查误杀，几乎不动。
- `retest8` / `retest9`：记录为回退后“能走一点”，README 未提供精确距离或时间。

README 对这些结果的总结属于既有实验记录，不等于当前 Git baseline 已重新复现。详表见 `docs/experiments.md`。

## Known Problems

- 代表 run 普遍存在转向占比高或移动有限的问题。
- `retest6` 记录了走廊检查误杀。
- README 表明后加的 `turn_thre`、走廊检查、方向惯性和净空惩罚已被回退；当前 baseline 不包含这些改动。
- 部分文档仍描述旧的 `go2_robot_sdk`/WebRTC 工作流，而当前根 README 的原生 UTLiDAR 主线不需要 PC 侧 LiDAR driver。
- 运行路径、标定文件、网络接口和实时/仿真时间设置存在环境耦合。

## 初始接管基线（历史，2026-10-06）

- Git baseline：`e5e133a chore: import existing GO2 project`
- 分支：`main`
- 代码基线：根 README 所述的 2026-08-31 打包版回退代码。
- README 记录的回退参数包括 FAR `robot_dim=0.7`、`converge=0.30`，local `obstacleInflate=0.2`、`stopDisThre=0.2`、`useCost=false`，WP5 `min_goal_dis=0.8`、`arrive_dis=0.35`、簇门槛 2、`risk_weight=0.3`、`min_obs_dis=0.2`。

## Current Questions

- 当前 baseline 能否在全新 Ubuntu 22.04 / ROS 2 Humble 环境完整构建？
- 本轮静态分类已给出 24 个 package 的主链/辅助/仿真/旧或替代流程角色；仍需运行时确认混合用途 package 的实际启用组件。
- 实时运行时 `/clock` 是否存在，Point-LIO 的 `use_sim_time` 是否与主线一致？
- GO2 当前固件、ROS domain、CycloneDDS QoS 和网卡配置的正式支持矩阵是什么？
- `imu_calib_data.yaml` 的权威版本、生成方法和部署位置是什么？
- 主链真实 TF 树及各点云/里程计 frame 是否一致？
- README 的 retest 指标如何计算，分析脚本与原始日志能否复现这些数值？
- safety gate 的 Sport lease、ARMED 和 API 成功能否对应到可测量的真实运动？

## Standalone D435 Perception Workstream

- 独立工作空间：`camera_ws/`；不加入现有 `code/` 导航 build/launch。
- ROS 2 package：`go2_science_perception`，提供只读 camera diagnostics 和 CameraInfo inspector。
- 2026-09-28 宿主只读枚举确认 USB `8086:0b07` D435 和 UVC device nodes 存在。
- 2026-09-28 已从 ROS 2 Ubuntu Jammy 官方 APT 仓库安装 librealsense 2.58.4 与 RealSense ROS wrapper/description 4.58.4；D435 firmware `5.12.7.150`，USB 3.2。
- 官方 wrapper 和 `camera_ws` 完整 launch 均已实测 RGB、raw depth、aligned depth、CameraInfo 与相机内部 `/tf_static` 数据；640x480 color/aligned-depth 尺寸一致，实际 frame 已记录在 `camera_ws/docs/architecture.md`。
- `camera_ws` 独立 build 成功，4 个 package tests 全部通过，完整 launch 运行超过 30 秒并干净退出。CameraInfo/metadata 约 30 Hz，但 Python 大图像订阅约 RGB 18 Hz、aligned depth 8 Hz且有短暂 freshness 告警，持续端到端 30 Hz 尚未验证。
- camera optical -> GO2 vehicle -> SLAM map 的只读TF2转换原型已实现；外参仍为 `PROVISIONAL / NOT CALIBRATED`，且未接导航。
- 独立 `blue_surface_center` 已回退会稳定选择侧面的上部55% ROI，改为完整蓝色target mask点云、最多4平面顺序RANSAC；先执行35°法向硬过滤，再按内点数和残差选择。30秒实测204帧、56 VALID（27.5%）；主要拒绝为 `bad_normal=118`，其次 `low_inlier_ratio=19`。实测存在最大侧面误差85.860°而第二候选误差11.070°并被选择的帧，证明最大平面没有被直接当作上表面；当前比例仍不能称为稳定。输出 frame 为 `camera_color_optical_frame`，尚未做 GO2/map 转换。
- 2026-09-29 已加入基于完整mask有效深度中位数的距离/尺度自适应：`<1.0 m -> 0.008 m`、`1.0–1.5 m -> 0.012 m`、`>=1.5 m -> 0.018 m`；RANSAC最少内点为 `max(30, ceil(point_count * 0.03))`。`perception_quality` 综合 mask面积、有效深度比例、平面内点比例和法向误差，只有 `GOOD` 发布精确相机系3D中心，`MARGINAL/UNRELIABLE` 仅保留2D/debug观测。
- 同日完成三档各15秒受控实测：近档0.284–0.286 m为49/49 `UNRELIABLE`（全部 `bad_normal`）；中档1.194–1.215 m为62/63 `MARGINAL`、1/63 `UNRELIABLE`；远档1.795–1.877 m为63/67 `MARGINAL`、4/67 `UNRELIABLE`。三档均无 `GOOD`，因此均未发布精确3D中心。该结果证明分段阈值和质量门控在实机数据上生效，不证明精确3D识别已稳定。
- 当前源码default和运行YAML的 `mount_pitch_deg` 已统一为30.0°。上述三档数据采集时仍使用45.0°，当前30°配置的近/中/远表现为 `NOT VERIFIED`。
- 独立 `coarse_target_locator` 已扩展为多蓝色目标：HSV/aligned-depth候选先以3D RANSAC剔除大范围高支持竖直蓝墙，再对剩余区域按2D连通与深度层聚类；每目标在既有topic连续发布一个PointStamped。2026-10-02实机静态验证最终窗口中，仅蓝墙为0发布，蓝墙+单目标稳定为1，蓝墙+3个不同大小目标稳定为3；不接导航，且同色目标与墙近乎共面时仍可能不可分。
- 2026-10-03针对蓝墙残片补充全平面距离过滤：RANSAC仍负责拟合，拟合成功后以
  `wall_rejection_band_m=0.08 m`删除全部近墙蓝色有效深度点，并以参数化
  `target_min_wall_separation_m=0.10 m`检查剩余cluster median墙距。固定3目标+1蓝墙
  的35秒稳定窗口为134帧×3点；完整链另一个35秒窗口为160帧×3点，manager只确认
  T3/T4/T5且未继续增长。启动期仍创建过两个未确认瞬态track，近共面真实目标边界
  尚未验证。
- 2026-09-29完成0.5/1.0/1.5/2.0 m四档实机粗定位：实际median Z分别为0.487–0.488、0.941–0.954、1.467–1.513、1.918–1.998 m；发布统计37/37、60/61、38/38、64/64，frame均为 `camera_color_optical_frame`。这是功能与连续发布证据，不是外部真值测距精度标定。
- 新增独立 `camera_target_to_map`，按输入PointStamped原始时间戳查询TF2，发布 `/science/target_coarse_point_map` 与RViz Marker。暂定 `vehicle -> camera_link` 外参为 `(0.36,0,0.12) m`、RPY `(0,+30,0) deg`，并复用RealSense的camera_link到optical TF；`coarse_target_locator`未修改。
- 只读实机组合明确未启动FAR/WP5/local planner/path follower。有效A/B试验中蓝色目标静止、GO2由用户人工移动，map目标均值三维差约14.9 mm（25与32样本）；整轮2347次转换、TF失败0。该单次结果支持“本场景基本稳定”，但外参仍为 `PROVISIONAL / NOT CALIBRATED`，不能作为导航接入或标定精度证据。
- 2026-10-02在固定目标下完成名义正面/左/右平移组及同一位置中间/左转/右转航向组复测。平移组位置均值最大XY差为94.0 mm，旋转组航向均值最大XY差为26.9 mm；人工摆位未用外部量具验证，且map转换存在3262次时间戳TF失败，仅成功发布样本进入统计。该结果表明暂定外参/map链仍有明显视角相关偏差，不支持导航接入；完整数表和边界见 `camera_ws/docs/experiments.md`。
- 同日完成TF future-extrapolation专项：旧日志372条可解析样本的`point-latest TF`差为0.000229–6.590080 s。D435实机launch现显式设置Point-LIO `use_sim_time=false`，map转换对原始点时间戳执行最多8 s有界重试并逐次记录point/latest/delta，绝不替换为latest TF。固定目标五视角复测内部计数为3579/3579转换成功、0最终失败（1次启动期miss随后精确时间重试成功）；该结果不改变外参仍为`PROVISIONAL / NOT CALIBRATED`及暂不接导航的结论。
- 本工作流仅包含蓝色区域测试检测，不包含通用目标分类、导航接入或 GO2 运动控制。
- 2026-10-02新增`science_target_manager`导航侧第一阶段离线实现：按同帧多点做
  map XY一对一关联（默认0.20 m），5帧确认，近期median位置和稳定ID；基于经TF
  转到map的`/base_state_estimation`选择最近未访问目标并生成0.40 m standoff可视化。
  confirmed/selected/standoff均有RViz输出，但没有`/goal_point`、FAR/WP5或运动命令
  接口。源码确认`/base_state_estimation`继承Point-LIO的`camera_init` header，因此
  管理器不会直接将其当作map。单元/静态验证已完成；真实ROS graph、连续TF和真机
  多目标Marker运行链已于2026-10-03完成约2分钟只读实机验证：RViz同时收到多个
  confirmed、selected、standoff及visited重选更新，未接`/goal_point`或运动链。
  现场候选较多并累积到至少T18，因此每个稳定ID是否都对应独立真实目标仍为
  `NOT VERIFIED`。
- 2026-10-03在3个固定真实目标、GO2人工前后/左右/原地转向/斜移条件下完成8个
  15秒稳定窗口。原始T1–T3的最大跨阶段均值XY差为34.62 mm，存在厘米级视角相关
  偏差；旋转/斜移又产生T6/T11/T14/T17，最终7个confirmed ID对应3个真值，且
  selected转向额外track T14。standoff几何仍保持距selected 0.40 m，但目标语义不再
  可靠；当前结论仍为不得接入FAR或运动控制。
- 2026-10-06仅为`science_target_manager`增加candidate/confirmed/stale生命周期、
  超时删除、连续多帧confirmed合并和active-confirmed-only选择；关联半径仍为0.20 m，
  FAR/WP5/规划/控制接口均未接入。48项package tests通过且独立build成功。三固定目标
  的8阶段人工移动重跑中，每个15秒稳定窗口均收敛为3个ACTIVE CONFIRMED，selected
  未引用STALE/已删除ID，standoff持续0.400 m；三个空间目标跨阶段均值最大XY差为
  54.02–64.28 mm。实机发生STALE/REMOVED但未触发MERGED，因此合并实机触发仍为
  `NOT VERIFIED`；完整证据见`camera_ws/docs/experiments.md`。外参仍为
  `PROVISIONAL / NOT CALIBRATED`，仍不得接FAR或运动控制。
- 2026-10-08将manager完成语义改为实时track加map空间访问记忆：新增
  `visited_positions`、默认`visited_radius=0.15 m`及
  `/science/mark_selected_visited` Empty触发；新track ID只要位于visited半径内仍被
  selected排除。54项package tests和隔离build通过；固定3目标的`3 -> 2 -> 1 -> 0`
  真机验收因`enp12s0 NO-CARRIER`、无GO2 map输入而为`NOT VERIFIED`。D435仅确认USB
  枚举存在。蓝色检测、蓝墙过滤、camera_to_map及无运动边界未改。
- 2026-11-03完成3个固定目标实机复测。用户接受视野变化导致目标短暂刷新；可见稳定
  窗口中map变换连续且standoff为0.400–0.402 m。visited人工触发完成
  `3 -> 2 -> 1 -> 0`；原T2遮挡后以新ID T70重新确认，并在距历史visited位置
  0.028 m处继续显示`visited/ignored`，证明过滤不依赖track ID。遮挡和启动瞬间仍会
  产生蓝色误检，属于已知限制。
- 同日增加最小一次性`science_goal_bridge`，只接受有限`map` frame standoff并锁定首个
  目标转发到`/goal_point`。56项package tests和独立build通过。FAR/terrain/localPlanner/
  pathFollower在不启动WP5、安全门、武装或Sport输出的条件下启动；FAR自带RViz因注册
  第二个`/goal_point` publisher而被停止。启动污染轮的一次goal发布已作废，干净轮因
  额外T6确认并抢占selected而在bridge启动前停止。因此有效FAR收目标、`/way_point`、
  `/path`及`/cmd_vel_raw`均仍为`NOT VERIFIED`。

## Next Recommended Work

仅建议调查，不在本轮实施：

1. 在无真机条件下建立 package、node、topic、frame 和参数的完整静态清单。
2. 在干净 ROS 2 Humble 环境执行依赖检查与可重复 build，记录精确命令和失败项。
3. 分类现有测试：纯静态/单元、合成数据、依赖 rosbag、依赖 GO2 真机。
4. 使用一组明确来源的 rosbag 做只读离线回放，验证 Point-LIO 到 `/cmd_vel_raw`，不启动真实运动输出。
5. 完成硬件安全清单后，再设计分阶段 dry-run 和低速真机验证。

## 2026-11-06 — 独立单目标接近（本轮授权范围）

- 分支 `feature/d435-perception-workspace` 新增 `science_target_approach`；独立预览
  launch强制dry-run，既有视觉/蓝墙/Point-LIO/FAR/WP5算法不改。manager仅新增带原始
  观测时间、track寿命、ID及visited的只读JSON快照，供稳定锁定和可见性判断。
- D435已移至GO2主机，Foxy独立相机驱动构建完成，经用户对时、限定DDS peer和话题
  remap后PC实收RGB约14.9 Hz、aligned depth约14.4 Hz。源事实与实测见
  `camera_ws/docs/experiments.md`；外参仍 PROVISIONAL / NOT CALIBRATED。
- 初轮dry-run被蓝色误检抢占nearest；用户后续将检测参数调为s_min=110、
  min_component_area_px=250，Codex同步YAML并重编译，155/155及151/151帧各只有1目标。
  随后T1稳定锁定、预览最高0.30、检测/定位输入超时锁存均通过。
- 默认0.80 m停靠、最高0.30 m/s、轨迹寿命门槛、明确ID确认、TF/定位/状态/竞争发布者
  检查、终态锁存和JSONL日志已实现；配置关闭gate的0.20 m/s最低速度抬升。
  默认保持预览；真实模式仅在用户后续逐次明确授权下经gate输出。
- 用户随后明确授权0.10 m/s短时及0.30 m/s上限完整接近。完整轮锁定T1，最高Move命令
  0.294 m/s，约5.73 s、距离0.917 m时视觉丢失并停车，后续里程计距离0.875 m。
  用户确认方向正确、已停住、无明显异常；这不是0.80 m定距停车通过。窗口位姿差分
  速度峰值0.494 m/s（真实性待核），实际速度不超过0.30 m/s仍NOT VERIFIED。
- 真实控制器和gate已停止、raw publisher=0；相机/SLAM/检测链按用户要求保留。
  后续定距停车验收及阶段三仍未完成，APPROACH_ODOM真实模式继续由代码拒绝。

## 2026-11-06 — MAP锚点与相机原点接近（858f75a后续）

- 已实现稳定中位数MAP锚点、按空间位置跨ID重关联及小幅视觉修正；视觉间断不删除
  锚点，APPROACH_ODOM在实时Point-LIO/相机TF有效时闭环，默认25 s/2.0 m上限。
- 停车参考改为相机camera_link原点的MAP XY，真实带时间戳TF包含旋转后的安装偏移；
  0.10 m容差，忽略Z。默认预览不发raw，显式--execute入口自动录像/锁定后执行一次。
- 旧轮0.917 m视觉FAULT主要与0.38 s输出间隔叠加0.5 s时间门槛有关，不支持直接判定
  出视野/面积不足/depth失效。HSV/墙过滤算法未改，增加原检测轮廓、mask和诊断输出。
- 本轮第一轮kp=0.5于35 s超时，MAP误差34.2 cm，用户实测35 cm。kp提高至1.0后，
  一轮在7.827 s进入camera XY STOP，误差8.29 cm，最后9.43 cm；用户确认约10 cm处
  停住且效果良好。只记录本场景粗验收通过，正式标定/重复精度仍NOT VERIFIED。
- 实际RGB检测MP4已保存，成功文件为camera_ws/records/approach_20261106_210841_496906.mp4，
  同名JSONL含逐帧、控制及用户实测数据；完整204/204帧解码通过，数据本机保留不提交Git。
- 指令最大0.30，Sport反馈峰值0.316 m/s，不能把指令限速写成实际速度严格上限证明。
  结束撤下实机controller/gate、raw publisher=0；保留只读感知及独立日志窗口。

## 2026-11-07 — 移动搜索与自动MAP接近

- 在原`science_target_approach`内增加SEARCH，沿启动MAP朝向最多1 m，初始0.15、
  用户后续要求改为0.20 m/s；一个raw发布者切换到原接近控制，不启动FAR/WP5/局部规划。
- 最近1 s至少5个不同原始时间戳MAP观测、最大两点XY散布<0.10 m自动锁定中位数锚点，
  无3 s寿命等待、不依赖像素静止；保持原视觉空间修正、失视觉Point-LIO闭环和急停。
- 实机A为18 cm用户实测超时失败；B为ALIGN超时；C为未确认目标搜索超时。
  D使用0.20搜索、kp_linear=1.5、kp_angular=2.0、braking_latency=0.30，自动切换后
  9.800 s以camera XY=0.0927 m触发到达STOP，用户确认“接近10cm完成”。
  最后MAP读数0.1130 m，Sport反馈峰值0.3503 m/s，不能宣称精密停车或实际严格限速通过。
- 实际检测MP4完整241/241帧：`camera_ws/records/approach_20261107_172248_186678.mp4`，
  同名日志及run摘要本机保留；详细失败/成功证据见`camera_ws/docs/experiments.md`。
- 独立构建、24项针对性控制检查通过。暗光识别使用用户已实测结论，不重复测试、不调HSV。
  外参仍PROVISIONAL，重复精度/实际严格速度上限/路线终点物理停车仍NOT VERIFIED。

## 2026-11-07 — 重复搜索失败调查与会话清理

用户重复轮`run_1794043913372908704`末段确有RGB检测，但控制锚点未恢复视觉更新。
原锚点(3.697,0.694)m，后续原始MAP点偏移数十厘米并产生T89/T90，超出原空间门槛。
接近8.800s、失视觉累计里程计路径2.003m时触发`odometry continuation limit`，当时
相机XY误差1.113m，不是到达停车。MAP偏移根因仍待查；未调HSV或放宽门槛。
已按用户要求停止PC只读链及远端D435，原始332帧视频/日志保留。详情见camera_ws实验记录。

## 2026-11-08 — 固定物块 MAP 漂移诊断准备

基于9d657b8，新增观测记录（原始RGB/Depth stamp、camera XYZ、实际MAP TF、
关联分支原因）、只读JSONL采集与统计脚本；不修改HSV、Point-LIO、关联门槛或控制计算。
旧录像332帧完整解码，175检测/80有效帧实际时差绝对P95约67.19ms；旧逐帧MAP/TF/
Point-LIO原始姿态不足，且/tmp感知日志已丢失，底层根因仍Unknown。
A/B2/C诊断数据已采集：A最大/P95 2.51/2.27cm；B2物块注释14.00/4.07cm；
C候选13.72/5.96cm（最大点身份被录像tile遮挡，NOT VERIFIED）。严格B/C运动和全程
可见性未达，C用户只部分返回。未复现旧23–39cm已核实同物块漂移，根因仍Unknown。
GO2时钟已恢复，正式采集前同连接SSH测得约-6.1ms。TF算术一致不证明外参或定位准确。
详见camera_ws/docs/map_drift_diagnostics.md。不得将本次时钟异常反推为前轮根因。

## 2026-11-08 — 5cm目标STOP_CONFIRM流程（已实现，6秒版待测）

基于a8fb268新增SEARCH→STOP_CONFIRM→原APPROACH，源匹配相机距离≤1.8m连续候选
触发停车，反馈/位姿停稳后仅收新样本，至少5帧/0.6秒源跨度、散布<0.08m取中位锚点。
用户最后指定确认总时限6秒，包含制动/停稳/采样，冷却0.75秒恢复原有界搜索；其余
HSV、Point-LIO、关联/ODOM保护和原接近控制不变。30项相关检查、package构建及lint通过。
已完成4轮真实运行：初始可见2秒版用户实测9cm成功；移远后有前端无候选失败；2秒版
搜索发现但停稳超时；4秒版两次确认超时后按搜索总时限停车。严格不可见起步的重复
成功率尚未证明提高。6秒版按用户要求今天不再运动，明天实测，NOT VERIFIED。
D435先前未枚举问题重插后恢复；曾出现PC USB过流日志，连接状态需明天复核。
上一轮中断的10mm分析未纳入本轮，备份在/tmp/go2_10mm_pending_before_stop_confirm_20261108。

## 2026-11-09 — 用户指定远目标搜索参数与复测

SEARCH配置/执行入口默认改为5 m、60 s，速度由0.20改为用户确认的0.25 m/s；
确认总限保持6 s，SEARCH专用ODOM路径上限5 m、失视觉时间25 s，安全门仍0.30 m/s。
31项针对性检查、独立package build、修改Python文件flake8与diff检查通过。
USB实收D435；GO2时钟退回2022年，经用户本地输入sudo密码恢复后才启动传感器。
第一轮仍用1 m搜索，未获有效候选，路线结束，用户确认停稳，双录像237/237帧解码。
第二轮按5 m/0.25运行，进度约3.413 m时段后定位超时；用户报告中途切换遥控器并
确认停稳。同段网卡carrier-changed/DOWN、DDS写失败；不将网络事件全部归因遥控器。
该中断轮未进入STOP_CONFIRM，非6秒确认验收；双录像327/327帧完整，runner收尾
中断未生成run摘要，另存明确标注的post-hoc audit。恢复链路后用户另授权重试。
完整结果/原始证据见camera_ws/docs/experiments.md；5 m边界与6秒完整流程尚不可
仅据参数和构建认定通过。未修改Point-LIO、HSV、空间关联或安全门。

恢复链路后第3轮第一次6秒确认超时，第二次总耗时3.400 s确认成功（停稳1.900 s、
采样1.500 s、6帧、散布5.26 cm），随后原接近约10.00 s到达STOP。首次MAP XY误差
7.59 cm、收尾8.80 cm；用户明确确认停稳并实测9.5 cm。两份录像均560/560帧完整。
本场景6秒确认/接近链取得一次用户确认结果；未行满5 m搜索或5 m ODOM边界，
这两项真实边界与重复精度仍NOT VERIFIED，不能外推成功率提升。

后续用户报告当前版本“实测两次都可以”，并要求推送。已记录为用户现场反馈；所述
两次未逐一关联原始run并审计，不补写距离/耗时或定量重复精度。实验实现为07292cd，
参数保持搜索5 m/0.25 m/s/60 s、确认6 s、SEARCH专用ODOM路径5 m/25 s。

## 2026-11-09 — 可复用YOLO训练与独立PC推理

来源：远端基线e15c66c、源码、PC针对性测试/构建、官方模型/合成数据消息检查；
新分支feature/reusable-yolo-training。新增vision_training的普通YAML训练/评估/PT与
ONNX导出、视频抽帧/图片导入/显式人工审核/拍摄组划分/数据检查和复现快照。
ROS新增独立yolo_detector，权重内类名及可选类别筛选，独立2D JSON/调试图/可靠深度
粗点输出，不发布原HSV点、map或运动接口；旧HSV源码/配置/launch保持不变。

PC独立venv：RTX4060 Laptop 8GB、驱动595.99.02、Python3.10.12、torch2.5.1+cu124、
Ultralytics8.3.40、numpy1.26.4/OpenCV4.10；实际CUDA矩阵运算通过。
16项新增检查和53项旧HSV/搜索/接近检查通过；仅构建go2_science_perception。
同一train.py用不同YAML完成单/双类别合成fixture各1 epoch，评估/子集/导出实际执行；
PT/ONNX模型在隔离DDS域各5种RGB-D情形检查通过，源图时间戳与类别选择可追溯。
上述合成fixture与官方COCO模型不代表蓝色立方体性能。

历史30视频中10份raw共207.8秒，已提取213张未审核图像；尚无人工审核YOLO标签，
所以没有训练blue_cube模型或真实mAP/困难组结果。真实D435 YOLO流、3D准确性、持续
速率、连续漏检和Orin/NX/TensorRT为NOT VERIFIED。
安装/导入观察到间歇Python/标准库异常，限制CPU亲和/线程后部分检查通过；
环境长期稳定性及根因未确认，详见vision_training/VALIDATION.md，不假定已修复。
当前已有bridge/10mm分析的无关未提交改动不纳入本任务。

## 2026-11-09 — 真实blue_cube首次训练前的人工标注准备

来源：最新远端19c840d、本地文件/ZIP校验、213张全量视觉质量筛查与31对候选对比。
原213张和pool manifest保持不变，213张全部解码，无精确重复/损坏图；第一批送标
181张，31张同视频近重复不送标，1张严重重影暂存hold。保留可辨认的模糊/边缘图，
例如左缘截断的107也保留，不能把部分出画误当无效。质量筛选不是人工标签审核。
本地annotation_v1/images_for_cvat.zip逐张内容hash核验通过；可追溯筛选计划与报告、
CVAT指南已入仓，媒体仅本机保留，没有创建标签或review回执。
当前0份标签/0回执，且仍仅一个拍摄组，**不具备真实训练条件，本轮停在人工标注**。
没有训练真实blue_cube模型、没有新增mAP。建议补两个小规模独立组覆盖蓝色干扰物、
多目标、受控弱光/边缘/距离，优先静态/手持采集，不安排GO2运动。
17项训练/数据与标注包检查通过；初次pytest收集遇既有解释器异常，按既有CPU亲和
16–19条件重跑通过，未重装环境、未全仓构建。train.py、competition.example.yaml与
全部受保护的HSV/定位/manager/搜索/运动源码配置保持不变。

## 2026-11-10 — 20张人工标注YOLO pilot

来源：CVAT ZIP、本地哈希/框图检查、RTX4060真实训练与独立验证输出。
最新远程基线ac76fe9已核对。独立blue_cube_pilot_v1只使用20份人工标签和精确匹配图片，
整视频划分14 train/6 val，无test；同实验室场景，不声称独立拍摄会话。原181图哈希不变。
指定50轮/patience15，实际20轮早停，best第5轮；CSV累计16.2077秒。
独立val P=0.003333、R=1.0、mAP50=0.00996661、mAP50-95=0.00712589；conf=0.25
下6验证/12未标注定性图片均0框，无可信成功检测案例，不能部署。未标注图不算定量测试。
90次预热后GPU推理均值6.385ms，同步predict墙钟8.969ms，不代表ROS延迟/持续FPS。
18项针对性测试通过；默认三组审核保持严格，只有显式--pilot允许train/val。
本地best/last/曲线/对比图/快照见vision_training/runs/blue_cube_pilot_v1；
报告vision_training/reports/blue_cube_pilot_v1.md及版本/指标YAML入仓。
未重装框架、未全仓build、未启动ROS/GO2；真实部署、跨场景及连续推理NOT VERIFIED。
下一步补独立会话和弱光/边缘/尺度/干扰物人工标注，再建立新版本微调。

## 2026-11-10 — 第一版YOLO失败诊断（不训练）

来源：b7a2e46基线best/last真实GPU推理、20图标签裁剪复核、CSV与固定8.3.40源码。
best在conf0.01下训练TP/FP/FN=3/973/11、val=1/427/5；conf≥0.05两组均0框。
主要支持A：尚未可靠学会训练集，非已经充分拟合后的单纯泛化失败；last虽低阈值训练TP增多，
高分输出仍全部误检。未发现明显错标签/类名/路径错误；val6张全为面积<32²的小目标。
发现默认100batch预热大于本次80batch总量，按源码静态重建仅17个更新调用点（非实测有效更新数）。
建议先补多样人工标注，下一版主基线从官方yolo11n.pt开始，同时适配小数据更新预算；不默认续best。
报告vision_training/reports/blue_cube_pilot_v1_failure_analysis.md；本机完整证据analysis_v2。
21项针对性测试通过；仅新增诊断代码/测量图/报告，无训练、框架更改、全仓build或GO2连接。
唯一失败根因及改进后的效果尚需受控验证，不把推断写为已证实因果。

## 2026-11-10 — blue_cube_pilot_v2配置修正训练

来源：最新Push 3ea9fbc、RTX4060真实训练/推理、optimizer hook与保护文件哈希。
仍使用同20图14/6及相同官方YOLO11n初始化；nbs=4、epochs/patience=80，warmup=3沿用V1。
80轮完成，best第70轮；320batch累积始终1，实测optimizer.step313次（另7次AMP跳步）。
conf0.25：V1 best train0/0/14、val0/0/6；V2 best13/0/1、4/0/2；V2 last14/0/0、4/0/2。
conf0.05：V2 best train14/1/0、val5/0/1；last14/1/0、6/0/0。上述均TP/FP/FN、IoU≥0.5。
结论：已能拟合14张训练图；同场景6张val不足以证明真实泛化，边缘小目标仍低置信度。
训练调用87.85秒，CSV累计74.69秒；24–27轮出现10个NaN验证loss单元，原始记录未改，
之后恢复有限。不能声称全程数值稳定，异常唯一原因未证实；本轮没有追加训练/搜索。
236个V1/数据保护文件及库trainer源码hash未变。30项针对性检查通过；未全仓build或启GO2。
详见vision_training/reports/blue_cube_pilot_v2.md；权重/全量预测本机runs/blue_cube_pilot_v2。

## 科学目标手机视频送标（2026-11-13）

- 用户再次确认手机实拍、三个不同场景，来源phone_real；不冒充D435。
- 三视频完整解码8696帧，A/B/C保留444/339/260张，共1043张；6包174×5+173。
- 源帧逐图内容匹配、尺寸、ZIP完整性/唯一性检查通过，6项针对性离线测试通过。
- 无自动标签、审核或split；先人工标第一批。原600张合成数据只读保留，manifest匹配历史hash。
- 交付与导回：vision_training/reports/science_targets_video_20261113/REPORT.md。
- 未训练/启动机器人；正式三类效果和D435泛化NOT VERIFIED。

## 首轮三类混合GPU训练（2026-11-14）

- 来源：用户明确确认157张标全/类别正确、17张全部无目标；原CVAT缺17标签，保留原ZIP并在规范化副本补空标签。
- 174张成功导入，真实框331/33/94（color_block/compact/spiky），83张多目标。
- 新版本独立复制：train480合成+117手机A/C，val120合成+57手机B；采集组和图片hash无跨split泄漏。
- RTX4060实际100轮、batch8/nbs64、warmup边界225 batch，7500批/971 optimizer step，用时948.53秒；best第100轮。
- 分来源mAP50/50-95：合成val0.9949/0.9447；手机B val0.3673/0.2355；手机A/C训练拟合0.9946/0.8517。
- 手机B各类AP50为0.8374/0.0538/0.2108，conf0.25时compact TP0/FN33；未达到可部署三类效果。
- 37项检查、全量导入检查、实际评估、三类失败/成功拼图；原600张合成图标签1200项hash复核一致。
- 真实compact训练样本仍缺；手机验证只有一个场景，独立最终test与D435泛化NOT VERIFIED。
- 报告：vision_training/reports/science_targets_mixed_v1/README.md；权重/大数据本地runs/science_targets_mixed_v1。
- 未修改ROS2/MAP/Point-LIO/导航/抓取；无硬件操作。

## 第二轮优先送标与960兼容性（2026-11-14）

- 远端起点0691a5d；剩余869张逐图hash/读回检查通过，A370/B282/C217。
- 定制送标180张（C140/A40/B0），原尺寸/文件名/来源保留，ZIP与五页拼图本地annotations/science_targets_round2_priority。
- 按目视时间窗覆盖灰色岩石候选、不同大小/角度、方块多目标和较暗/模糊背景；不自动区分compact/spiky，不能保证补齐compact。
- 所有180张pending，无标签/新审核/训练；完整B继续验证组。原best/last和首轮评估hash核对一致。
- PT960 GPU推理通过：实际矩形1×3×544×960峰值allocated71.88MiB，方形warmup94.82MiB，均不含CUDA上下文/ROS等额外显存。
- 固定640 ONNX需另行960导出并匹配元数据；本轮未导出或修改ROS2。960训练显存/精度、ROS实流仍NOT VERIFIED。
- 报告和预览：vision_training/reports/science_targets_round2/README.md。2项测试、ZIP/逐图检查通过。


## 科学目标混合V2实训与验证（2026-11-14）

- 来源：最新远端454d544、用户确认161张标全/类别正确、ZIP/逐图hash检查、100轮RTX4060 GPU实训和同57张B验证。
- 第二轮实际按名称重写ID 0→0/1→2/2→1，161 positive、0 confirmed negative、19 pending；新增框56/0/122。19张缺标签清单/缩略图已保存，未补空标签或纳入训练。
- 独立science_targets_mixed_v2：758train（480合成+278真实A/C）、177val（120合成+原57B）；原600张、V1数据/权重/评估6523文件hash和集合不变。V2审核池独立，后续选图须排除其161张reviewed。
- V1 best初始化YOLO11n/640/batch8/nbs64/warmup3，100轮、1958.46秒、9500batch/1233更新，best第100轮；49项离线检查、compile和dry-run通过。
- B验证V2 640 P/R/AP50：block .8666/.6958/.8178，compact 0/0/.0189，spiky .2810/.3333/.2318。conf.25逐类TP/FP/FN为139/21/85、0/17/33、14/27/46；目标未达标。
- 同一640权重960推理AP50为.8294/.1215/.3039，spiky Recall .50；conf.25 compact仍0TP，总FP65→82。未做960训练，不继续盲调；真实compact训练0框、新122个spiky仅1个小目标，优先补独立真实训练场景。
- 保留HSV，不改Point-LIO/MAP/导航/抓取；D435实时RGB-D/三维关联、最终独立test仍NOT VERIFIED。完整报告/证据：vision_training/reports/science_targets_mixed_v2/README.md；本地best在vision_training/runs/science_targets_mixed_v2/baseline/weights/best.pt。


## YOLO单方块MAP接近接口（2026-11-14，仅代码）

- 核对远端视觉abb83f5、实际集成caf6ab6及对应源码；新增approach集成入口，YOLO仅color_block/V2固定hash，HSV互斥回退。
- 继承原camera_target_to_map精确时间转换；完整YOLO帧选最近可测方块并锁定，仅其单点进入原Manager/Approach。深度无效/歧义不更新，两个方块≤0.40m保守拒绝。
- 原ROI中位深度改选实际有效像素，补RGB/Depth/CameraInfo时间/frame检查；不把框中心当几何中心。原控制器/Manager/转换器/0.10m配置hash未改，未进入S1/抓取流程。
- 主仓库入口转发实际集成模式，补齐PC校时包装器参数；原full/dock路径不改。12项针对性检查、单包构建、指定venv导入、launch参数及互斥检查通过。
- 用户最新要求先完成代码；本轮无设备连接/实时预览/运动。D435定位误差、实时画面、到位距离/停止原因、单/多方块真实运行全部NOT VERIFIED。
- 入口与限制：camera_ws/docs/color_block_map_approach.md；预览命令bash go2_test.sh --mode approach --preview --view。后续先静态误差核验，再另授权运动。


## 第三轮YOLO候选数据（2026-11-15，不训练）

- 远端起点cd6dc9c；实际找到4段新手机视频及10段历史GO2 raw，完整解码9787+3117=12904帧。用户确认手机不同场景/独立拍摄会话；相近水泥背景的外推限制仍保留。
- 第1/2段及历史9365帧V2离线推理，仅辅助筛选；第3/4段不推理留出。最终446候选=310训练候选+50验证候选+86最终测试候选，不建立正式split。
- 历史130图逐图质量预览，21张可辨模糊方块且运动日志支持；5张边界不清uncertain排除送标；旧20张人工标签复制复用不再送标。
- 优先CVAT包305图：round3_priority_batch_001.zip153张、batch_002.zip152张，位于vision_training/annotations/science_targets_round3；原尺寸，仅图片、无预测标签/自动negative。类别固定0block/1compact/2spiky。
- 新训练视频模型提示compact18张/spiky128张，真实类别数量Unknown待人工确认；预测小目标17张/边缘93张也不是GT。第3/4段136图不推理、不送当前优先包；旧B57仍是开发验证非最终test。
- 446张原视频重新解码/hash/尺寸核验、ZIP完整性及9项针对性检查通过；9708个保护文件未变，不改600合成、335真实、19pending、旧20标签或V1/V2模型。
- 代码/统计/少量预览见vision_training/reports/science_targets_round3/README.md；原视频、批量图和ZIP留本机。本轮无训练、部署、ROS/运动/抓取；新GT类别/正式划分/V3性能NOT VERIFIED。

## 2026-11-15 第三轮305张人工标签与YOLO11n V3

- 来源：用户CVAT及明确确认46张negative；305图=259positive/46negative，新增框365/0/140，原ZIP保留、规范化副本补空，按名称映射。
- 独立V3 1063train/177val，原600合成、V2数据及同57张B保留；新视频3/4不入train，历史GO2同lab组，旧20图未重复追加。
- RTX4060完成100轮640训练（V2 best初始化），43.35分钟；best SHA256 db74e03e4073574fb91b9fe3cb404c0f256b13c6451c014639818471e186e391。
- 同B开发验证block P/R/AP50=0.7646/0.8702/0.8736，spiky P/R/AP50=0.8231/0.3000/0.4664；完整三版本、AP50-95及TP/FP/FN见vision_training/reports/science_targets_mixed_v3/README.md。
- 历史10段3117帧三版本预测保留原RGB时间戳；90张连续片段审核包已建，用户选择稍后人工补标。预测空档不是GT漏检；真实最长漏检/恢复、留出新场景和独立GO2泛化NOT VERIFIED。
- 真实compact训练仍0框；不替换现有V2，不启动ROS或运动，不改MAP/Manager/Approach/抓取。54项视觉检查通过，旧6173文件hash不变。
