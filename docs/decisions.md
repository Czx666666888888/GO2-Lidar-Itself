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

## ADR-008 — 在线offset跟踪方案评审（待用户确认，2026-11-13）

- **状态：Proposed / needs confirmation；未实施、未部署、未测试。**
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

## 2026-11-20 — avoid_ws 使用最小条件动作规则

- **Decision / 用户本轮明确要求：** 将独立避障简化为传感器式 0/1 触发，不改变其他导航或 FULL 架构。
- **源码：** 固定前/侧检测框；0.20 m/s 直行；前 25 cm 触发停稳后选向转弯；侧 15 cm 触发连续反侧 20°弧线。
- **边界：** 取消覆盖、净空和旋转扫掠门槛；未检出回波不能等同实际无障碍；保留数据/控制权/急停互锁。
- **Evidence：** `avoid_ws/README.md`、controller/lidar/node 和根入口离线测试；实机 `NOT VERIFIED`。
