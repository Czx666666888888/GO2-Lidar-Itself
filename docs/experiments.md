# Experiment Record

## 2026-11-14 实测前Humble出口回调兼容修正

本轮授权的一次测量尚未发前进指令。先后记录：外层PC时间入口未透传35cm参数（已修复）；
本机密码需交互；远端重启导致S1/Socket消失（用户另授权恢复，已恢复ZERO_HOLD并原对时）。
随后run_1794659502828542193仍在IDLE、仅StopMove时节点退出：
TypeError: ScienceTargetApproach._sport_output() missing required positional argument info。
当前安装的Humble executor只传消息正文，take_message元信息仅含source/received时间，没有GID。
修正为单参数回调，以当前graph唯一go2_safety_gate发布者归属记录；有其他发布者即明确停止。
不伪称逐包GID验证，不改ROS执行器，不加起步基线门槛。32项针对性检查及新增单参数回归，
独立本机ROS域87/localhost真实Move和StopMove回调测试通过；该域未连接GO2。
单包构建与安装一致性通过，随后才继续尚未执行的同一轮授权测量；未增加物理测量次数。


## 2026-11-14 前向标定测量修正（第1步，未运动）

最新Push基线7e9c865。复核20:15:13的run_1794658509091411854：最高控制请求约0.1800m/s，
窗口3.4763cm触发零速时原始定位已约7.8cm，后续触及20cm原始位移保护；
stop_extra_m=0.1212m混入窗口迟滞，不能当作真实制动距离，更不能生成0.20标定profile。

本次只修正受控响应测量，未增加起步基线门槛、未改导航架构/原5m预算/SEARCH/S1动作：
- 只读订阅既有/api/sport/request，按DDS publisher GID只接收go2_safety_gate的输出，
  API1008速度/API1003零速逐条写原JSONL；不增加任何速度发布者。
- 实际输出达到参考0.20后，记录该时刻原始vehicle pose为测量段起点；
  再前移2cm即请求零速，使用原始位姿而非延迟的移动中位数触发停车。
- 原始/窗口位移并列，独立记录控制器zero_requested、outlet_zero、实际停稳证据和确认时刻，
  记录请求/出口峰值、原始pose采样时间、明确的越界原因。
  观察到DDS请求不等于电机实际执行，物理位移仍需现场标记/固定录像核对。
- 未达到参考速度、未观察到出口归零、未完成停稳均不算完整响应测试。
  physical_review_required=true、calibration_accepted=false，不自动填审核标志。
- 默认20cm/8秒、横偏3cm/转角3度保护保留。新增仅响应测试可用的
  --response-distance-limit参数（0.10～0.50m），不影响普通dock/full或5m续行预算。
  离线计划明确打印所选边界，普通dock/full禁止使用该覆盖。

原模型a_up=.15、a_down=.30、latency=.30，达到0.20的理想加速路程.1333m，
再测2cm，模型减速.0667m加延迟.06m，合计约.28m。
这只是原模型估算，非已测制动值；因此准备35cm边界的下一轮明确测试请求，
未经新运动授权不执行，不自动扩张默认20cm限制来绕过失败。
建议现场前方至少1m空旷、急停在手、有人监护、S1健康ZERO_HOLD，
准备地面标记及固定录像/卷尺确认实际位移；本轮最多先做一次。

32项针对性检查（响应阶段/出口解析/原停靠接口）、语法、关键lint、单包构建与安装读回通过。
启动器既有F841未使用变量告警已识别为历史代码，没有混入无关清理；其余关键lint通过。
原速度/位置/时间/反馈失效保护保留；本轮未启动GO2或S1动作。
真实响应、制动profile和独立dock均NOT VERIFIED，尚未宣称完成标定。


## 2026-11-14 移除响应测试额外的单次基线门槛

最新run_1794658282823146261、full_1794658268393168074，20:11:25首次故障为
forward response check requires stable trusted pose baseline。安全门只有StopMove、没有前向Move。
该门槛是前轮增加的起步瞬间定位窗口判定；未记录具体blocker，不能从10Hz快照反推
完整内部高频窗口。故障时体速约0.0187、相机速度0.0175m/s，Sport速度近零；
保留快照重算窗口体/相机速度约0.0244/0.0230m/s、半径约1.41/1.44cm均通过，
不足以确定启动回调瞬间是哪个内部指标不通过，不能说成0.20m/s执行失败。
按用户明确要求删除begin_response_check中的额外一次性稳定基线拦截，不新增等待流程。
保留原新鲜度/TF/Sport反馈、唯一速度出口、加速度、20cm原始位姿/8秒、3cm横偏和3度航向保护。
测试参考0.20，窗口位移记录和实际停稳判据不改；数据有歧义仍不能生成可信标定profile。
21项针对性检查（含新增不完整额外窗口不再起步FAULT回归）、语法/diff与单包构建通过；
安装源码逐字节一致并更新内容指纹。本轮未发送任何运动指令，实际响应仍NOT VERIFIED。


## 2026-11-14 19:48:07停靠失败分析与最小控制修复

基线：fetch最新Push为0444d33cbfdc77296f766bf024ea5db49ab90de1。本轮无实机运动。
用户后续明确：0.15m/s不能移动，0.20m/s可稳定移动；这是用户现场反馈，不是本轮控制实验。
用户最后批准停车距离4～6cm均可：本轮配置验收改为<=6cm，航向10度及停稳0.5秒不变。
SEARCH、STOP_CONFIRM、vy/wz计算、Point-LIO/时间映射/D435算法、S1轨迹/协议未修改。

### 原始证据及可确认边界

本轮失败run_1794656817517152442，full_1794656799004878859；
控制/安全门/定位相机日志、视频检测sidecar已读取，原始文件哈希见
[camera_ws分析证据](../camera_ws/analysis/docking_20261114_194807/evidence.json)。
原记录里程4.993814666m；可恢复的10Hz转换后vehicle位姿折线4.614532627m，
同段净位移0.498057865m，Sport速率积分0.530736006m。
原始逐帧Point-LIO Odometry/完整高频pose回调未保存，不能把10Hz快照冒充原始Point-LIO。
新的0.5秒移动中位数、2cm分辨率保留锚点估计，在同份10Hz记录上为2.541488015m；
相对4.9938m减少2.452326651m，但它包含采样差异、滤波及未分辨往返，**不是确认的噪声米数**。
Sport积分也不是独立真值。没有外部物理测量，实际位移/定位抖动/漂移各占多少无法定量确认；
不能写实际只走了0.53m或余下全是噪声。新证据一致性检查在同份记录失视觉后约6.20秒
已可检出路径证据持续矛盾；这是离线检查结果，不是改控制后的物理轨迹预测。

### 六次STOP_WAIT完整序列

逐点vehicle/target MAP XY、航向误差、原始Sport vx/vy/wz、控制指令、
body/camera/Sport速度与停稳计时阻塞原因，见
[完整CSV](../camera_ws/analysis/docking_20261114_194807/stop_wait_series.csv)。
六次退出均为当前距离超过原6cm退出阈值，没有航向超限、2秒等待超时或目标锚点变化：
- 第1次：3.019cm → 6.363cm，持续0.5s，退出航向误差7.44度，目标MAP变化0。
- 第2次：3.816cm → 6.859cm，持续0.4s，退出航向误差7.13度，目标MAP变化0。
- 第3次：3.906cm → 6.325cm，持续0.5s，退出航向误差7.09度，目标MAP变化0。
- 第4次：3.838cm → 6.557cm，持续0.4s，退出航向误差6.98度，目标MAP变化0。
- 第5次：3.885cm → 6.987cm，持续0.6s，退出航向误差7.37度，目标MAP变化0。
- 第6次：3.733cm → 7.202cm，持续0.4s，退出航向误差7.16度，目标MAP变化0。

六段共28个STOP_WAIT状态样本，仅13个在原4cm内；只有一个样本启动过停稳计时，
未持续0.5秒。目标锚点在这些段固定，波动来自vehicle MAP位姿；Sport几乎静止，
支持检查定位抖动/漂移和姿态摆动，现有证据不能独立排除真实超调。
精调483个样本中479个正vx<0.15m/s、443个Sport合速度<0.01m/s。
配置0.18只是平面请求下限；safe=sqrt((a*latency)^2+2*a*clearance)-a*latency，
随后还受实测速度制动、方向分解和向量加减速限制。例旧4cm容差下、a=.30/latency=.30，
6cm距离时safe约0.089m/s；斜向运动的vx会更低。不能把请求下限当作最终速度保证。

### 失视觉

最后有效检测色彩图戳1794656834492360840，中心(621.845,452.285)，在640x480画面的
右下边缘，深度约0.389m。之后442份检测，343份no blue candidates，99份聚类后无有效目标。
原始录像抽查100/200/300/600/950帧，后段目标已不在可见画面内；相机仍连续出帧。
不能把目标失效说成USB掉线，也不能只改识别阈值假装恢复。
旧逻辑没有dock视觉时效停止，允许追旧MAP直到5m路径预算耗尽；最终首次故障时误差
5.8819cm、视觉年龄53.2203秒、剩余预算6.19mm，未STOP/未释放导航，所以没有StandDown。

### 改动及验收边界

1. 新dock-only docking_motion时间窗估计：移动中位数、保留锚点累计慢速持续运动和
   可分辨往返；保留raw/windowed/Sport/未分辨差额。Sport为零不删除LIO位移，
   两个审计窗口累计路径矛盾超过8cm时停止并报localization untrusted，5m预算不变。
   2cm以下/窗口内快速往返不能保证分辨，明确标未分辨；本轮不宣称精确真实里程。
2. STOP_WAIT使用0.5秒位姿趋势及2cm包络估计实际停稳，仍要求新鲜Sport/TF/定位及
   原速度/角速度阈值。单帧越界不立即重启：退出还需窗口中心超外圈并持续0.3秒。
   终验必须当前<=6cm、<=10度且实际停稳持续0.5秒；外圈8cm仅零速滞回，不是成功容差。
   超6cm持续2秒仍恢复必要修正。新增每次退出原因和计时器阻塞原因JSONL字段。
3. 用户要求短步打开：左右dock overlay的dock_forward_step_enabled=true。
   正向参考速度改为用户确认有效的0.20m/s；不把它当全局速度下限，停车可为0。
   仍复用ForwardStep MOVE/SETTLE和实际投影位移、原制动/加减速出口。
   缺少真实制动profile时，必要正向精调直接零速故障并提示标定，不再持续无效低vx。
   左右/后退不加死区补偿；vy/wz原控制计算保留。profile速度必须匹配当前0.20，
   旧0.15记录不能冒充有效数据。已启用不等于已经完成制动标定。
4. 有效视觉超过3秒即停车并要求重新确认目标，不无限追旧MAP；不自动重新SEARCH。
5. 复用唯一入口的forward-response-check，按配置缓升至0.20，窗口测得2cm请求零速，
   原20cm/8秒、横偏3cm/航向3度保护保留。起步需稳定可信定位基线，测量不等于v*t；
   保留原始20cm位姿边界，不能因滤波延迟漏掉越界。无稳定基线不发送测试速度。

103项针对性检查（其中8项新增回归）、Python语法/关键lint/diff通过；
第一次未带ROS PYTHONPATH检查有1项导入失败，补齐原环境后全部通过。
第一次沙箱colcon执行器阻塞后仅中断本轮构建；正常环境顺序单包构建成功，
安装的6份控制源码及左右配置逐字节一致，已更新内容指纹，后续不重复构建。
本轮没有连接新SDK、机械臂使能、GO2行走、趴下或抓取。
**下一步只具备0.20m/s受控响应测试的软件条件；完整dock仍待制动实测profile和定位证据验收。**
当前forward_step_profile_file为空，不能承诺普通dock能完成全部末端动作。


## 2026-11-14 修复零位恢复后导航许可锁存

### 本轮授权后的实际部署结果

用户确认可靠支撑、空间无干涉、急停在手，授权仅S1升级及安全重启。
旧PID3365已空闲/零位/通信正常，按原SIGTERM流程正常退出；确认USB0释放后升级。
兼容层v2_usb01_feedback_2已部署，新右侧PID5633唯一占用USB0。
启动时已在原容差内，日志STARTUP_ZERO_SKIPPED_ALREADY_AT_ZERO；未执行非零位回零。
连续12次真实Socket采样均ZERO_HOLD、busy=false、at_zero=true、navigation_allowed=true、
communication_state=OK；最大关节绝对偏差0.0268944rad。旧状态许可false的阻塞已解除。
保留新版保持服务；未启动GO2行走、趴下或抓取，full仍未实机验收。
证据本机camera_ws/records/s1_nav_recovery_9448301/（before.json、after_samples.json），
运行日志camera_ws/records/s1_nav_recovery_9448301_run.log。
采样SHA256：`b327eeddf782d5af1c64ef5e81eb8a763458d59c039e4df181f30c1c2af948e1`。超限后再次恢复的连续硬件过程未人为制造，已由离线回归覆盖。


来源：用户异常输出、原Socket只读状态和适配源码。J1先为-0.0803009rad（超过0.08rad），
随后已回到-0.0268944rad，at_zero=true、readiness_state=ZERO_HOLD且通信正常，
但navigation_allowed仍false。根因为snapshot只在超限时写false，恢复后不重算true。
修复为每次有效反馈按ZERO_HOLD、busy=false、at_zero及无服务故障重算许可；
通信异常、忙碌、真实FAULT、仍超限均不放行。原0.08rad容差、轨迹、协议、GO2验收不变。
兼容层版本v2_usb01_feedback_2，升级只接受原版/已部署上一版/本版精确hash，
按hash保留升级备份；运行中不改服务文件或创建第二所有者。
51项针对性离线检查通过，覆盖超限→恢复、忙碌/故障/通信异常阻断及上一版hash升级；
Python语法、关键lint、diff检查通过。本改动不涉及ROS包构建。
远端原服务仍运行旧代码，升级后真实状态恢复NOT VERIFIED，需支撑确认后安全重启S1。
本轮未发送回零、抓取或GO2运动指令。未把软件修复写成硬件部署通过。

## 2026-11-14 修复S1进程清单误报历史版本

用户full日志在connection_and_inventory阶段拒绝/home/unitree，未进入导航/姿态/抓取。
源码确认原pgrep按整条命令行匹配，而SSH检查代码自身包含左侧脚本名称，可能匹配其外层shell；
同时原版本判断用cwd，绝对路径启动的真实v2服务也可能被误报。
现按/proc读取Python实际脚本操作数，排除shell、python -c/-m和其他脚本参数中的名称；
相对入口结合cwd解析，绝对入口直接解析，按真实scripts父目录判断v1/v2。
仍拒绝真实旧版/多所有者/不一致端口，未删除安全检查，未停止任何进程或重启机械臂。
42项针对性离线检查、语法/关键lint/diff通过；远端执行完全相同的只读清单返回
arm=[]、task=[]、camera=[]、USB0唯一存在、D435/D455枚举正常，无历史S1进程。
原始清单：本机camera_ws/records/s1_v2_inventory_fix_20261114/inventory.json。
本轮没有运行dock/full，运动、自动回零及抓取效果仍NOT VERIFIED。

## 2026-11-14 S1 v2 独立部署、兼容层及真实零位保持

- 基线：fetch后本地/远端ce20ba909dbecc99a6858ede143e9c79b8cf4c73。
- 用户原ZIP：S1_arm_integration_20261114_auto_v2.zip，14,098,177 bytes；
  SHA256 `892c7be031fd52bceaf335caeb189567de364afecaa20d21de1a2f047afee3d9`。
  CRC、路径、BUILD_INFO和原SHA256SUMS通过；不提交ZIP或大日志。
- 实际安装：GO2伴随主机.18 `/home/unitree/s1-arm-control-v2`；
  保留旧目录，部署前后旧版scripts/s1_control/config/calibration共111文件哈希一致。
  原先没有S1服务/串口所有者，故没有停止旧保持服务或抢占。
- v2专用适配`v2_usb01_feedback_1`：默认auto兼容唯一USB0/1，全服务锁+原设备锁；
  设备inode/rdev改变即通信异常。原v2文件备份`.v2-compat-upstream/`，
  `S1_V2_COMPAT.json`核对适配哈希，其余使用原清单；不修改抓取算法、轨迹或协议。
- 用户本轮明确授权右侧启动及原startup_zero，排除GO2运动/趴下/抓取。
  只读反馈有效：q约[-0.037194,-0.023460,0.000572,-0.005150,0.026894,0.049020]rad，
  温度17～20°C；原0.08rad零位容差内。服务日志正确输出
  `STARTUP_ZERO_SKIPPED_ALREADY_AT_ZERO`，随后ZERO_HOLD。PID3690、USB0唯一所有者。
  8次连续Socket采样以及只读定位检查后均busy=false、navigation_allowed=true、at_zero=true、
  communication_state=OK。最后最大绝对关节角约0.0132rad。保留该服务，不为清理而撤保持。
- Python3.10复用已有环境及SDK，没有重装依赖。右侧服务真实启动；
  左右`verify_handoff.py`离线预检和包完整性通过，仅右侧访问机械臂。
  D455序列231622302209，15次彩色/深度frameset返回，启动初期有重复帧/编号重置，
  随后双流递增；这是短时接口证据，不代表抓取识别验收。D435序列213622070598。
- 原生LiDAR只读5秒：74云/1250 IMU样本、25个LidarState，健康检查PASS。
  伴随时钟原为2022年，操作者通过原sync_go2_clock.py终端完成对时；
  未改时间映射算法。短时链检查首次因动态导入缺少scripts路径失败，修复后重跑。
  8秒有效采集：D435、Point-LIO、连续输入、时间映射、时间戳限制、精确base/camera TF全通过；
  119个track快照，0个coarse/map_point/selected，没有CONFIRMED目标；
  整体为`NOT VERIFIED: exact_target_map, confirmed_selected_target, fresh_confirmed_target`。
  无数据不足以判断目标未检出的原因，未改感知算法。检查自有相机/定位进程已正常收尾。
- 软件交接核对：只用status/grasp，没有zero RPC；同侧健康服务复用，不重新初始化；
  STOP释放导航→新鲜停稳反馈→StandDown反馈确认→原抓取→新结果ZERO_HOLD→StandUp逻辑保留。
  41项针对性离线检查通过（实际ZIP适配分支、串口/状态接口、原姿态序列）；
  Python/Bash语法、diff检查通过；按内容缓存仅构建变化单包成功，非全仓测试。
- NOT VERIFIED：非零位startup_zero实物运动、USB1换号、导航运动中ZERO_HOLD、真实dock、
  StandDown/抓取/StandUp。SDK未提供独立电机包时间戳；关节反馈不等于独立物理测量。
  本轮无dock/full运动授权，未运行。右侧Y=0.41m与v2建议约0.31m仍需实际位置和支撑验收。
- full目前只具备S1服务和软件交接条件，未具备本场景完整验收：仍需有效MAP目标、
  新授权独立dock通过、机械臂路径/机身支撑/抓取位置确认，再由用户执行full。
- 原始记录本机`camera_ws/records/s1_v2_deploy_20261114/`；其中right_startup.log、
  zero_hold_samples.json、readonly_hardware.log、final_remote_verify.log。
  定位证据`camera_ws/records/time_20261114_191821_s1_v2_readiness_retry/report.json`，
  SHA256 `e82f6ef5852108726b33a2ac2e2985e2fb69329734049aae8fa8c3bbb35b4541`。本轮失败首次报告及重试日志均保留。


The entries below are a structured transcription of the root `README.md`. No rosbag was replayed and no metric was recalculated during creation of this document.

## Evidence Rules

- **Recorded observation:** text already present in the repository README.
- **Interpretation:** only the interpretation stated in that README; no new causal claim is added.
- **Raw data:** the local `rosbags/` directory is ignored by Git. Raw rosbag stored locally / not tracked in Git.

## retest4

- **Experiment ID:** `retest4`
- **Configuration:** Current README associates the representative runs with the 2026-08-31 rollback code, but does not provide a per-run parameter manifest.
- **Observed result:** Duration 78 s; moved 0.82 m; 94% of the time was spent turning.
- **Interpretation recorded in existing documentation:** The robot could move, but turning dominated the run.
- **Raw data availability:** Raw rosbag stored locally / not tracked in Git (`rosbags/full_chain_a_retest4_20260901_153430/` observed during repository import).
- **Verification status:** `NOT VERIFIED` in this documentation cycle.

## retest5

- **Experiment ID:** `retest5`
- **Configuration:** Per-run details are not present in tracked documentation.
- **Observed result:** Duration 258 s; recorded as the day's farthest run at 2.96 m; 67% of the time was still spent turning.
- **Interpretation recorded in existing documentation:** Best recorded distance that day, but excessive turning remained.
- **Raw data availability:** Raw rosbag stored locally / not tracked in Git (`rosbags/full_chain_a_retest5_20260901_160622/` observed during repository import).
- **Verification status:** `NOT VERIFIED` in this documentation cycle.

## retest6

- **Experiment ID:** `retest6`
- **Configuration:** README identifies a corridor-check variant but does not provide a complete parameter manifest.
- **Observed result:** Duration 219 s; almost no movement.
- **Interpretation recorded in existing documentation:** Corridor checking falsely rejected motion (“走廊检查误杀”).
- **Raw data availability:** Raw rosbag stored locally / not tracked in Git (`rosbags/full_chain_a_retest6_20260901_173037/` observed during repository import).
- **Verification status:** `NOT VERIFIED` in this documentation cycle.

## retest8

- **Experiment ID:** `retest8`
- **Configuration:** README says this run was after rollback; no independent configuration manifest is tracked.
- **Observed result:** “能走一点” (some movement); no duration or distance is documented in README.
- **Interpretation recorded in existing documentation:** Movement returned after rollback.
- **Raw data availability:** Raw rosbag stored locally / not tracked in Git (`rosbags/full_chain_a_retest8_20260901_175103/` observed during repository import).
- **Verification status:** `NOT VERIFIED` in this documentation cycle.

## retest9

- **Experiment ID:** `retest9`
- **Configuration:** README says this run was after rollback; no independent configuration manifest is tracked.
- **Observed result:** “能走一点” (some movement); no duration or distance is documented in README.
- **Interpretation recorded in existing documentation:** Movement returned after rollback.
- **Raw data availability:** Raw rosbag stored locally / not tracked in Git (`rosbags/full_chain_a_retest9_20260901_180538/` observed during repository import).
- **Verification status:** `NOT VERIFIED` in this documentation cycle.

## Missing Evidence

- The referenced `today_representative.md` and `today_analysis_20260901.tar.gz` are not tracked in this repository.
- Exact metric calculation methods, per-run commit hashes, parameter dumps and environment metadata are not recorded in the root README.
- Future experiment entries should include commit hash, parameter snapshot, command, host/robot configuration, bag checksum, safety conditions and analysis command.
