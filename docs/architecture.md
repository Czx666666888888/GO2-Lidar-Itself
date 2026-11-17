# Current Architecture

## 2026-11-17 恢复横向目标（当前配置）

用户确认恢复左右S1横向±32cm、误差±2cm；前后±6cm、双轴历史锁存和原交接保持。
这是参数恢复，无架构变化；详见camera_ws/docs/dual_axis_arrival_20261117.md。


## 2026-11-17 后续横向配置更新（后续已撤回）

当前左右S1横向目标±31cm、误差±1cm；前后误差仍±6cm，双轴历史锁存和
原STOP/姿态反馈交接保持。仅参数更新，证据见camera_ws/docs/dual_axis_arrival_20261117.md。


## 2026-11-17 双轴历史到位（源码/离线验证）

左右抓取停靠分别锁存预测S1前后误差≤6cm、横向误差≤2cm（Y目标±32cm），
两方向均有到位样本后才进入原航向/停稳确认，原STOP→释放控制→姿态反馈交接不变。
已锁存方向制动、未到位方向继续修正；新任务和FAULT清除。普通MAP/SEARCH不改。
证据与历史记忆/当前误差边界见camera_ws/docs/dual_axis_arrival_20261117.md；真机NOT VERIFIED。


## 2026-11-17 S1 v4接入（源码/离线确认）

PC full入口经s1_v4_delivery固定原ZIP与兼容层，使用独立s1-arm-control-v4。
同一服务处理左右grasp及显式dump，反馈失效不入队；上层grasp只接受对应任务的新结果。
D435/D455交接、多目标循环和原姿态反馈链保留；左右停车目标±32cm共享给自动选侧。
当前没有返回卸载区导航，dump尚未成为full末尾的自动阶段。
细节和验证边界见camera_ws/docs/s1_final_fusion_v4_20261117.md；真机NOT VERIFIED。


## 2026-11-14 PC完整任务的YOLO接入（源码确认）

`go2_test.sh --mode full`经PC时间包装器进入原Workflow：共用V2资产验证，
原launch的detector=yolo提供类0方块/RGB-D→精确时间MAP→原Manager；
随后原SEARCH/STOP_CONFIRM/抓取停靠执行。成功STOP并释放唯一导航控制权后，
原s1_task_host/s1_task_sequence执行StandDown确认、S1抓取、ZERO_HOLD、StandUp确认。
新增的是编排参数连接和就绪检查，定位、目标管理、停靠、S1算法未重构。
approach仍独立只接近到camera XY10cm；full使用抓取位姿6cm/10度/0.5秒。
此处为源码/软件检查确认，整条实机链NOT VERIFIED。

本文档基于当前仓库静态阅读，描述能够从源码、launch 和脚本确认的结构。它不是当前硬件的运行证明；所有运行时关系仍需按 `TASKS.md` 验证。

## ROS 2 Packages

| Group | Packages | Repository role |
|---|---|---|
| GO2 application | `go2_keyboard_teleop` | keyboard control, recording, base odometry, WP5 exploration, safety gate |
| Localization | `point_lio_unilidar`, `transform_sensors`, `calibrate_imu` | sensor transform/calibration and Point-LIO |
| Terrain/local autonomy | `terrain_analysis`, `terrain_analysis_ext`, `sensor_scan_generation`, `local_planner` | terrain clouds, local paths, path following |
| Global/route planning | `far_planner`, `graph_decoder`, `boundary_handler`, `visibility_graph_msg` | visibility graph and global waypoint planning |
| GO2 interfaces | `unitree_api`, `unitree_go`, `go2_sport_api`, `go2_h264_repub` | messages and optional Sport/video utilities |
| UI/simulation/examples | `vehicle_simulator`, `visualization_tools`, `waypoint_example`, RViz plugins | simulation, visualization and operator tools |
| Integration | `ros_tcp_endpoint` | Unity/ROS TCP endpoint; not confirmed in the main GO2 chain |

## Mainline Data Flow

```text
GO2 onboard DDS
  /utlidar/cloud (sensor_msgs/PointCloud2)
  /utlidar/imu   (sensor_msgs/Imu)
        |
        v
sensor_transformer (transform_sensors/transform_everything)
  /utlidar/transformed_cloud
  /utlidar/transformed_raw_imu
  /utlidar/transformed_imu
        |
        v
laserMapping (point_lio_unilidar/pointlio_mapping)
  /cloud_registered -> remapped to /registered_scan
  /aft_mapped_to_init -> remapped to /state_estimation
        |                         |
        |                         +-> base_odom_node -> /base_state_estimation
        v
terrainAnalysis -> /terrain_map
terrainAnalysisExt -> /terrain_map_ext
        |
        +-> wp5_explore_node -> /goal_point
        |                           |
        +---------------------------v
                              far_planner
                    /goal_point + terrain + odometry
                              -> /way_point
                                   |
                                   v
                             localPlanner
                              -> /path
                                   |
                                   v
                             pathFollower
                           -> /cmd_vel_raw
                                   |
                                   v
                            go2_safety_gate
                    state/arm/stop/lease interlocks
                         -> /api/sport/request
                                   |
                                   v
                                  GO2
```

The diagram reflects the current `nav_launch.sh` and active launch files. Actual runtime delivery, rates and frame consistency are `TODO: verify`.

## Main Nodes and Interfaces

| Node/executable | Confirmed inputs | Confirmed outputs | Notes |
|---|---|---|---|
| `sensor_transformer` | `/utlidar/cloud`, `/utlidar/imu` | transformed cloud and IMU topics | Uses sensor-data QoS for native inputs; reads optional Desktop calibration file. |
| `laserMapping` | configured transformed cloud and IMU | `/registered_scan`, `/state_estimation` after launch remaps | Point-LIO node name in source is `laserMapping`. |
| `base_odom_node` | `/state_estimation` | `/base_state_estimation` | Applies configured sensor x/y offset. |
| `terrainAnalysis` | `/state_estimation`, `/registered_scan`, `/joy`, `/map_clearing` | `/terrain_map` | Terrain processing. |
| `terrainAnalysisExt` | `/state_estimation`, `/registered_scan`, `/terrain_map`, `/joy`, `/cloud_clearing` | `/terrain_map_ext` | Extended terrain cloud. |
| `wp5_explore_node` | `/terrain_map`, `/state_estimation`, `/far_reach_goal_status` | `/goal_point` | Maintains an accumulated grid and selects frontier observation goals. |
| `far_planner_node` | `/goal_point`, odometry and terrain inputs, `/update_visibility_graph`, `/joy` | `/way_point`, `/navigation_boundary`, `/far_reach_goal_status`, graph/debug topics | Launch remaps odometry/terrain inputs to current topics. |
| `localPlanner` | `/state_estimation`, `/registered_scan`, `/terrain_map`, `/way_point`, control/obstacle topics | `/path`, `/free_paths` | Uses precomputed path resources in `local_planner/paths`. |
| `pathFollower` | `/state_estimation`, `/path`, `/joy`, `/speed`, `/stop` | `/cmd_vel_raw` | Publishes `geometry_msgs/TwistStamped`. |
| `go2_safety_gate` | `/cmd_vel_raw`, `/lf/sportmodestate`, `/arm`, `/stop`, lease response | `/api/sport/request`, lease request | Default `dry_run=true`, initial state DISARMED. |

## Sensor Input

- Root README and `go2_lidar_preflight.sh` identify `/utlidar/cloud` and `/utlidar/imu` as native GO2 DDS streams.
- The PC-side mainline does not start an additional LiDAR driver.
- `go2_lidar_preflight.sh` checks exact message types and requires a sample from each stream.
- Network defaults in current scripts use CycloneDDS, ROS domain 0 and interface `enp12s0`; these are environment defaults, not universally verified requirements.

## Localization

- `mapping_utlidar.launch` starts `transform_everything` and Point-LIO.
- `utlidar.yaml` selects transformed UTLiDAR point cloud and IMU topics.
- Point-LIO outputs are remapped to `/registered_scan` and `/state_estimation`.
- The configuration currently contains `use_sim_time: true`.
- `TODO: verify` whether the live mainline supplies `/clock` and whether this setting is intentional for real hardware.
- `TODO: verify` the deployed IMU calibration file and the validity of fallback calibration constants.

## Mapping and Terrain

- `terrainAnalysis` consumes registered scans and state estimation, publishing `/terrain_map`.
- `terrainAnalysisExt` additionally consumes `/terrain_map` and publishes `/terrain_map_ext`.
- WP5 interprets `/terrain_map` intensity into unknown/free/occupied grid states.
- The exact semantic contract of intensity values across terrain nodes is partly encoded in implementation constants and is `TODO: verify` with runtime samples.

## Planning and Exploration

- WP5 publishes frontier observation goals on `/goal_point` and monitors FAR reach status.
- FAR consumes goal, odometry, local scan and terrain data, maintains a visibility graph, and publishes `/way_point`.
- local planner consumes `/way_point` and terrain/registered point clouds, publishing `/path`.
- path follower converts `/path` into `/cmd_vel_raw`.
- Graph decoder and RViz are included by FAR launch; boundary handler exists but is not directly started by the current main launch.

## Motion Control and Safety

- `go2_safety_gate` is intended to be the only authorized `/api/sport/request` publisher in the navigation chain.
- It contains DISARMED, ARMED and FAULT states; explicit arm/stop inputs; speed/yaw/acceleration limiting; command and robot-state timeouts; tilt checks; and optional foot-force checking.
- Non-dry-run mode includes Sport lease apply/renew processing before Move requests.
- `nav_launch.sh` defaults to dry-run but defaults `ARM_AUTO=1`; dry-run arming does not send motion requests.
- `nav_stop.sh` publishes Unitree StopMove and terminates named navigation processes.
- True physical stop behavior and exclusivity of publishers are `TODO: verify` on the target robot; code intent alone is not proof.

## TF / Coordinate Frames

Frames explicitly present in code/launch include:

- `body`, `utlidar_lidar_1`, `utlidar_imu_1` in sensor transform code
- `map -> camera_init` static identity transform
- `aft_mapped -> sensor` static identity transform
- `sensor -> vehicle` static transform using sensor offsets
- `sensor -> camera` static transform using camera offset and fixed rotation

Point-LIO message headers, odometry frame IDs and any dynamic TF broadcasts require runtime inspection. The authoritative complete TF tree is currently `Unknown / needs verification`.

## Build and Launch Entry Points

- Build: `bash code/scripts/deploy_local.sh`
- Environment: `source code/scripts/env_go2.sh <interface>`
- Read-only preflight: `bash code/scripts/go2_preflight.sh <interface>`
- Main chain: `bash code/scripts/nav_launch.sh --dry-run --no-arm`
- Stop helper: `bash code/scripts/nav_stop.sh`

These commands are sourced from repository scripts. Their success on a clean host is currently `NOT VERIFIED`.

## full三目标监督与相机/侧别交接（2026-11-15源码）

父监督器保持现有感知定位会话，串行启动3轮原停靠/抓取协调器。每轮控制器启用持续SEARCH及共参左右停车位评分，
锁定目标和侧别后仍走原STOP_CONFIRM、停靠及控制权释放。父进程暂停D435并确认释放后，
姿态适配器向现有S1 socket发送含grasp_side的grasp请求；唯一S1 Arm会话调用原侧别配置/轨迹。
原抓取/起立任务完成、D455释放和D435恢复确认后，协调器写round结果；父进程记录完成位置供下轮访问过滤。
第3轮后不再启动控制器；S1保持服务和恢复后的D435按原策略保留。three_grasps.json为软件任务计数，placement_confirmed仍null。
旧S1进程需正常退出后加载dual_side_5，不热修改串口所有者。完整实机流程NOT VERIFIED。


2026-11-16：现有SEARCH配置在STOP_CONFIRM用请求零速后的目标MAP收敛代替机身停稳前置条件。
开关stop_confirm_require_stationary=false；观测源时刻必须晚于本次确认开始，时间/空间关联
与manager CONFIRMED条件保留。原话题/接口、后续停靠与姿态/抓取数据流未变。
来源：源码及离线测试；实机NOT VERIFIED。


2026-11-16最新：上述单纯位置收敛模式已被用户新要求替代。当前SEARCH启用fast确认，恢复
实际速度/偏航+0.20秒停稳，复用Manager/SEARCH前史，停稳后3帧RGB-D唯一空间复核；
无可靠前史则5帧/0.60秒回退。6秒后一次2秒原地复核有界结束。后续MAP/S1/相机/三轮链路不变。
来源：源码、旧日志统计及26项针对性测试；新实机NOT VERIFIED。
