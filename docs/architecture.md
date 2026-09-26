# Current Architecture

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
