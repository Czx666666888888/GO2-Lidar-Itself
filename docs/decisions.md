# Architecture Decisions

This is a lightweight Architecture Decision Record. “Accepted” means the decision is visible in the current repository baseline; it does not imply fresh runtime verification.

## ADR-001 — Use GitHub as shared project state

- **Decision:** Track collaboration rules, current status, tasks, architecture, decisions and experiments in the repository.
- **Evidence:** `AGENTS.md`, `PROJECT_STATUS.md`, `TASKS.md` and `docs/` created for this project.
- **Reason:** Provide ChatGPT and Codex with one versioned, reviewable state source.
- **Status:** Accepted.

## ADR-002 — Consume the GO2 onboard UTLiDAR over DDS

- **Decision:** Use `/utlidar/cloud` and `/utlidar/imu` published by the GO2 over ROS 2/CycloneDDS; do not start an additional PC-side LiDAR driver in the current mainline.
- **Evidence:** Root `README.md`, `code/scripts/go2_lidar_preflight.sh`, `transform_everything.py`.
- **Reason:** Reason not documented beyond the statement that GO2 publishes these streams directly.
- **Status:** Accepted in current baseline; hardware availability is `NOT VERIFIED` in this documentation cycle.

## ADR-003 — Use Point-LIO for localization and registered point cloud

- **Decision:** Transform the GO2 sensor streams and feed them into `point_lio_unilidar`; expose registered scan and state estimation through launch remaps.
- **Evidence:** `mapping_utlidar.launch`, `utlidar.yaml`, `laserMapping.cpp`.
- **Reason:** Reason not documented.
- **Status:** Accepted; accuracy and real-time behavior need verification.

## ADR-004 — Combine FAR global planning with local planning/path following

- **Decision:** Feed exploration goals to FAR, FAR waypoints to `localPlanner`, and the local path to `pathFollower`.
- **Evidence:** `nav_launch.sh`, FAR/local planner source interfaces and launch files.
- **Reason:** Reason not documented.
- **Status:** Accepted in current baseline.

## ADR-005 — Generate exploration goals with WP5 frontier logic

- **Decision:** Accumulate `/terrain_map`, select frontier observation points and publish them as `/goal_point`; use FAR reach feedback and local BFS distance for reselection.
- **Evidence:** `wp5_explore_node.py`.
- **Reason:** Reason not documented.
- **Status:** Accepted in the 2026-08-31 rollback baseline; performance remains problematic according to README experiments.

## ADR-006 — Place a safety gate between path following and GO2 Sport API

- **Decision:** Route `/cmd_vel_raw` through `go2_safety_gate`, with DISARMED/ARMED/FAULT state, dry-run, limits, timeouts and lease checks, before publishing `/api/sport/request`.
- **Evidence:** `safety_gate.py`, `nav_launch.sh`.
- **Reason:** The source documents prevention of unarmed commands, stale-command continuation and loss of robot-state/lease control.
- **Status:** Accepted; physical safety performance is `NOT VERIFIED`.

## ADR-007 — Keep runtime data and build products out of Git

- **Decision:** Do not track colcon output, runtime logs, rosbag databases, generated analysis data, caches or local credentials.
- **Evidence:** Root `.gitignore`.
- **Reason:** Keep the source repository reviewable and avoid large/generated/sensitive artifacts.
- **Status:** Accepted. Raw experiment bags must be managed separately with an explicit location and checksum if used later.

## ADR-008 — 在线offset跟踪方案评审（已由ADR-009取代，未实施）

- **状态：Superseded by ADR-009；未实施滚动offset。**
- **操作边界：** 用户本轮要求先记录、确认后执行；本次只记录方案，不据此启动运动/机械臂。
- **实际实现基线：** 独立集成分支45eac367f7ca001a531985bc17dc03e6995934b5。
  当前transform_everything.py委托ImuTimeMapping：原始IMU的2秒窗口、至少200样本、
  接收时间减header的P5初始化b，初始化后固定；LiDAR/IMU共用该映射。
  用户引用的time_stamp_offset_set/首帧完整点云代码已不在该版本，不能按旧行号直接替换。
- **用户提出：** 只在transform_everything.py维护最近50帧完整点云recv-header中位数，逐帧更新b，
  共用于IMU；不改topic/frame/QoS/控制路径，实施后测试。
- **源码与实测风险：** 原始点云header对应扫描开始，Point-LIO继续加点内相对时间得到扫描末端。
  因此完整点云接收时刻减header包含扫描跨度，以及发布/网络/调度延迟；并非纯钟差。
  用该估计映射扫描开始后，再加点内时长，会把扫描结束推到接收时刻之后。
  中位数能压抖动，不能消除扫描跨度偏差；共用该b还会把偏差传给IMU。
  正常扫描中位约63.77ms；最新异常窗口约464ms、原生约2.07Hz/83.43%丢包。
  50帧只在15Hz时约3.3秒，在2.07Hz时约24秒，跟踪延迟随退化显著变化。
- **漂移数字核对：** 若漂移率真为780微秒/秒，30分钟累计为1.404秒，而非47ms；
  47ms约对应60秒。该漂移率未在本次重新测量，不作为已确认的GO2固定特性。
- **建议决策：** 不重新用完整点云recv-header估计b；如批准在线跟踪，优先在现有IMU
  观测窗口上持续估计共享b，明确有界更新、映射单调性与时钟突变处理，保留现有故障锁存/100ms保护。
  当前映射实现在imu_time_mapping.py，因此正式实现的文件范围需要按实际结构确定，
  不能承诺只加一个同名属性就能让现有IMU/cloud自动跟随。
  在线offset跟踪不等于独立UTC校准，也不能恢复原生雷达频率/丢包或修复伴随主机重启后的旧日期。
- **确认后最小验证：** 只构建受影响转换包，先确认无运动任务与原生健康，再经现有只读入口
  检查共享offset、IMU/点云/扫描末端与里程计时间、READY、单调性、精确TF/目标MAP；
  不启动运动、不修改Point-LIO/MAP/S1控制算法，不以短时通过宣称长期漂移已治愈。
- **证据：** 当前transform_everything.py/imu_time_mapping.py源码、
  camera_ws/analysis/go2_native_lidar_20261113及go2_blue_target_20261113实测记录。

## ADR-009 — 恢复历史legacy转换，保留当前控制功能（2026-11-13）

- **状态：Accepted，用户本轮明确授权实施。** 取代ADR-008的滚动offset提案；不开发新时间同步框架。
- **基线：** fetch后HEAD为45eac367f7ca；仅从a8f7c667874b提取时间转换公式，不回退整个仓库。
- **决定：** 默认legacy，首帧LiDAR回调b=本机ROS接收时间-header，运行期间固定；LiDAR/IMU
  都加同一b，点内时间不变，legacy不实例化/调用ImuTimeMapping的READY/FAULT/source_guard。
  启动前IMU等待首帧LiDAR建立b，避免沿用历史初始化前offset=0的另一时基；不改几何转换。
  imu/explicit模式保留可选，日志和状态明确模式。历史约57ms超前风险保留，不声称精确同步。
- **参数例外：** 用户本轮末尾明确要求future_tolerance由0.10改为0.15，已同时修改控制器默认值
  和YAML；odom_timeout=0.4s、停靠容差/速度/STOP/S1参数不变。IMU可选模式内部100ms保护未改。
- **运行边界：** 原生雷达不健康先停止，不用legacy掩盖原生低频。只读一键入口复用现有健康、
  新鲜度/TF/目标验收，不增加门限；运动需操作者另行执行原dock入口。
- **证据：** 历史/当前源码、两包构建输出、已安装文件与源码逐字节一致；本轮原生LiDAR8.15Hz、
  源端8.18Hz/35.36%丢包未通过，所以未启动legacy定位、D435/MAP或运动，实机恢复NOT VERIFIED。

## ADR-010 — 运行主机本地时间及NX本机入口（2026-11-13）

- Accepted：用户明确最终所有计算部署NX，PC仅开发/SSH/RViz。
- 每次启动用运行主机ROS系统时间建立会话offset，共享LiDAR/IMU，保持点内时间；不持久化offset。
- 本机入口无PC SSH/NTP/IP/clock；环境配置与算法分离，真实系统时间，记录漂移与突变故障。
- 突变停止数据输出并由既有控制超时请求停止；停止控制后重启定位建立新会话，不在运动中改地图。
- NX实查缺四包，完整定位及运动/S1编排NOT VERIFIED；本轮实际部署只读入口并验证缺包阻断。
- PC专用完整启动器保留开发用途；NX的execute明确阻断直到部署迁移验收，不能误用PC流程。

## ADR-011 — 抓取停靠持续修正及同入口full协调（2026-11-13）

- Accepted：用户明确要求不因无进展停车，随后要求一键接入自主搜索/趴下抓取/起立。
- 移除抓取停靠无进展故障分支，保留进展诊断、原位姿/停稳完成条件及独立定位/ODOM/急停保护。
- full复用既有SEARCH/STOP_CONFIRM/3DOF和S1动作链，接入同一PC校时暂停/恢复上下文。
- 停靠失败不交接姿态/抓取，抓取失败不自动起立。静态/定向检查通过，修改后实机未验收。

## 2026-11-13 微调最低10cm/s，取消停靠25秒时间上限

用户明确要求最低平移微调10cm/s，并取消触发微调后的25秒限制。
集成抓取停靠profile新增dock_min_linear_speed=0.10，作用于4cm容差外的合平移目标速度；
不是vx/vy各10cm/s。制动限速、实测速度制动、向量加减速和0.30m/s上限仍优先，
故最终指令可低于0.10m/s；进入容差后降到0并沿用10°/0.5s完成条件。
抓取停靠不再使用丢视觉25秒预算及基于剩余时间的拒绝，遥测remaining_time=null；
保留5m续行距离、剩余距离判据、定位/TF/反馈新鲜度、异常停车、人工急停、唯一控制出口。
SEARCH原5m/60秒及普通MAP时间逻辑不变；横向额外10cm保留，前移3cm尚未实施。
34项停靠/接口检查通过，覆盖10cm/s合速度/制动及25秒后继续、5m仍停车；无离线仿真。
本轮未实机运动，新速度和长时间丢视觉停靠实机NOT VERIFIED，实际可达性不据软件检查推断。
