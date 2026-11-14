# Experiment Record

## 2026-11-14 修复零位恢复后导航许可锁存

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
