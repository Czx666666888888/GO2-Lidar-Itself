# ROS 2 Interface Inventory

调查日期：2026-09-26
调查范围：commit `61af715` 的仓库静态内容，包括全部 `package.xml`、构建定义，以及当前主链相关源码、launch、YAML 和 shell 脚本。没有启动 ROS graph、DDS 或 GO2；运行时名称、实际 QoS 协商、频率和 TF 连通性仍需实测。

## Reading Conventions

- **Mainline:** 被 `code/scripts/nav_launch.sh` 直接启动，或由它包含的 launch 启动，并参与传感器到运动请求的数据链。
- **Auxiliary:** 构建/消息/调试/可视化/标定等支持组件，不是主数据链计算节点。
- **Simulation:** 包含仿真 executable 或仿真 launch。
- **Legacy/alternate:** 当前主脚本不使用，且源码或文档显示为旧的/替代流程。该标签不表示可以安全删除。
- **Source QoS shorthand:** `sensor_data` 表示 ROS 2 sensor-data profile（best effort、volatile）；`depth=N default` 表示只传入整数深度，使用 rclcpp/rclpy 默认可靠性和持久性。实际 DDS 协商是 `TODO: runtime verification required`。

## Package Inventory

共发现 24 个 ROS 2 package。

| Package | Path | Build type | Executable / node or artifact | Current role | Classification |
|---|---|---|---|---|---|
| `local_planner` | `code/autonomy_stack_go2_src/base_autonomy/local_planner` | `ament_cmake` | `localPlanner`, `pathFollower` | 局部路径选择与速度生成 | Mainline |
| `sensor_scan_generation` | `code/autonomy_stack_go2_src/base_autonomy/sensor_scan_generation` | `ament_cmake` | `sensorScanGeneration` | 从仿真/传感器数据生成 scan | Auxiliary; current real launch include is commented out |
| `terrain_analysis` | `code/autonomy_stack_go2_src/base_autonomy/terrain_analysis` | `ament_cmake` | `terrainAnalysis` | 注册点云的局部地形分析 | Mainline |
| `terrain_analysis_ext` | `code/autonomy_stack_go2_src/base_autonomy/terrain_analysis_ext` | `ament_cmake` | `terrainAnalysisExt` | 扩展地形图 | Mainline |
| `vehicle_simulator` | `code/autonomy_stack_go2_src/base_autonomy/vehicle_simulator` | `ament_cmake` | `vehicleSimulator`, `sim_image_repub`; real/simulation system launches | 当前真实主链的聚合 launch，同时保留仿真节点 | Mainline orchestrator; Simulation |
| `visualization_tools` | `code/autonomy_stack_go2_src/base_autonomy/visualization_tools` | `ament_cmake` + installed Python | `visualizationTools`, `realTimePlot.py` | 指标和地图可视化 | Auxiliary; main launch include commented out |
| `waypoint_example` | `code/autonomy_stack_go2_src/base_autonomy/waypoint_example` | `ament_cmake` | `waypointExample` | 文件式 waypoint/boundary 示例 | Auxiliary/example; not current mainline |
| `boundary_handler` | `code/autonomy_stack_go2_src/route_planner/boundary_handler` | `ament_cmake` | `boundary_handler` | boundary/graph 文件处理及 RViz | Auxiliary; not included by current main launch |
| `far_planner` | `code/autonomy_stack_go2_src/route_planner/far_planner` | `ament_cmake` | `far_planner` (source default node `far_planner_node`) | visibility-graph 全局规划与 waypoint 输出 | Mainline |
| `graph_decoder` | `code/autonomy_stack_go2_src/route_planner/graph_decoder` | `ament_cmake` | `graph_decoder` (source default node `graph_decoder_node`) | 解码/汇合 visibility graph | Mainline support, included by FAR launch |
| `visibility_graph_msg` | `code/autonomy_stack_go2_src/route_planner/visibility_graph_msg` | `ament_cmake`/rosidl | `Graph.msg`, `Node.msg` | FAR/decoder 自定义消息 | Mainline interface package; no node |
| `point_lio_unilidar` | `code/autonomy_stack_go2_src/slam/point_lio_unilidar` | `ament_cmake` | `pointlio_mapping` / `laserMapping` | UTLiDAR Point-LIO 定位与注册点云 | Mainline |
| `ros_tcp_endpoint` | `code/autonomy_stack_go2_src/utilities/ROS-TCP-Endpoint` | `ament_python` | `default_server_endpoint` | Unity/ROS TCP 服务 | Auxiliary/integration; not current mainline |
| `calibrate_imu` | `code/autonomy_stack_go2_src/utilities/calibrate_imu` | `ament_cmake` | `calibrate_imu` | IMU 标定工具 | Auxiliary; not current mainline |
| `goalpoint_rviz_plugin` | `code/autonomy_stack_go2_src/utilities/goalpoint_rviz_plugin` | `ament_cmake` | RViz shared-library plugin | 交互式 goal point 工具 | Auxiliary UI |
| `teleop_rviz_plugin` | `code/autonomy_stack_go2_src/utilities/teleop_rviz_plugin` | `ament_cmake` | RViz shared-library plugin | RViz teleop 面板 | Auxiliary UI; current mainline usage unconfirmed |
| `teleop_rviz_plugin_plus` | `code/autonomy_stack_go2_src/utilities/teleop_rviz_plugin_plus` | `ament_cmake` | RViz shared-library plugin | 扩展 RViz teleop 面板 | Auxiliary UI; current mainline usage unconfirmed |
| `transform_sensors` | `code/autonomy_stack_go2_src/utilities/transform_sensors` | `ament_python` | `transform_everything` (source default node `sensor_transformer`) | GO2 点云/IMU 旋转、过滤、时间戳和标定处理 | Mainline |
| `go2_h264_repub` | `code/autonomy_stack_go2_src/utilities/unitree_pkgs/go2_h264_repub` | `ament_cmake` | `go2_h264_repub` | GO2 H.264 图像重发布 | Auxiliary; current real launch node commented out |
| `go2_sport_api` | `code/autonomy_stack_go2_src/utilities/unitree_pkgs/go2_sport_api` | `ament_cmake` | `vel_ctrl`, shared client library | Twist/Sport API 桥与客户端库 | Alternate/legacy control path; not launched by current mainline |
| `unitree_api` | `code/autonomy_stack_go2_src/utilities/unitree_pkgs/unitree_api` | `ament_cmake`/rosidl | Request/Response message definitions | safety gate 与 GO2 Sport API 消息 | Mainline interface package; no node |
| `unitree_go` | `code/autonomy_stack_go2_src/utilities/unitree_pkgs/unitree_go` | `ament_cmake`/rosidl | GO2 state/command message definitions | `/lf/sportmodestate` 等 GO2 消息 | Mainline interface package; no node |
| `waypoint_rviz_plugin` | `code/autonomy_stack_go2_src/utilities/waypoint_rviz_plugin` | `ament_cmake` | RViz shared-library plugin | 交互式 waypoint 工具 | Auxiliary UI |
| `go2_keyboard_teleop` | `code/go2_keyboard_teleop` | `ament_python` | 9 console scripts including `base_odom_node`, `wp5_explore_node`, `go2_safety_gate` | 主链附加节点，以及键盘、记录、调试工具 | Mixed: Mainline + Auxiliary + legacy/alternate teleop |

Classification evidence: executables are defined in each package's `CMakeLists.txt` or `setup.py`; the active composition is `vehicle_simulator/launch/system_real_robot_with_route_planner.launch:11-54` plus `code/scripts/nav_launch.sh:115-184`.

## Node Inventory

“Required” refers to the intended current data/control chain, not a runtime liveness result.

| Effective node name | Executable / package | Launch source | Required | Inputs | Outputs | Parameter source | Remap / naming note |
|---|---|---|---|---|---|---|---|
| `base_odom_node` | `base_odom_node` / `go2_keyboard_teleop` | `nav_launch.sh:115-119` | Yes, for FAR odometry input | `/state_estimation` | `/base_state_estimation` | CLI: `sensor_offset_x=0.3`, `sensor_offset_y=0.0`; source defaults identical | No remap |
| `ps3_joy` | `joy_node` / external `joy` | `system_real_robot_with_route_planner.launch:11-15` | Launch-started; autonomy-chain necessity needs runtime verification | `/dev/input/js0` hardware | `/joy` (standard joy behavior; external package) | launch params | Runtime interface is external to repository |
| `transform_everything` | `transform_everything` / `transform_sensors` | `mapping_utlidar.launch:5` | Yes | `/utlidar/cloud`, `/utlidar/imu` | transformed cloud/raw IMU/IMU | constants plus optional `~/Desktop/imu_calib_data.yaml` | Source constructor name is `sensor_transformer`; launch name overrides it |
| `laserMapping` | `pointlio_mapping` / `point_lio_unilidar` | `mapping_utlidar.launch:7-26` | Yes | transformed cloud and IMU from `utlidar.yaml` | registered clouds, odometry, optional Point-LIO path, TF | `utlidar.yaml` plus inline launch overrides | `/cloud_registered -> /registered_scan`; `/aft_mapped_to_init -> /state_estimation` |
| `localPlanner` | `localPlanner` / `local_planner` | included launch at system launch `:25-34` | Yes | state, scan, terrain, `/way_point`, `/joy`, speed/boundary/obstacle controls | `/path`, `/free_paths` | `local_planner.launch`; system launch overrides autonomy/speed/offset arguments | No topic remap |
| `pathFollower` | `pathFollower` / `local_planner` | same include | Yes | `/state_estimation`, `/path`, `/joy`, `/speed`, `/stop` | `/cmd_vel_raw` | `local_planner.launch` | Direct Sport publisher is commented out in source |
| `terrainAnalysis` | `terrainAnalysis` / `terrain_analysis` | system launch `:36`; package launch | Yes | `/state_estimation`, `/registered_scan`, `/joy`, `/map_clearing` | `/terrain_map` | `terrain_analysis.launch` inline params | No remap |
| `terrainAnalysisExt` | `terrainAnalysisExt` / `terrain_analysis_ext` | system launch `:38-40`; package launch | Yes for FAR's extended terrain input | state, scan, `/terrain_map`, `/joy`, `/cloud_clearing` | `/terrain_map_ext` | package launch; `checkTerrainConn=true` passed by system launch | No remap |
| `far_planner` | `far_planner` / `far_planner` | system launch `:48` -> `far_planner.launch:16-35` | Yes | goal, odometry, terrain, scan, joy, update/status/file commands | `/way_point`, boundary, reach status, graph/debug/metrics | `far_planner/config/default.yaml` | Four input remaps; launch name overrides source default `far_planner_node` |
| `graph_decoder` | `graph_decoder` / `graph_decoder` | FAR launch `:52-55` -> decoder launch | Yes for current FAR graph exchange; exact necessity needs runtime verification | `/robot_vgraph`, save/read commands | `/decoded_vgraph`, decoder visualization | `graph_decoder/config/default.yaml` | Source input is absolute `/robot_vgraph`; launch's `/planner_nav_graph` remap does not match it; runtime node name is overridden from `graph_decoder_node` |
| `far_rviz` | `rviz2` / external | `far_planner.launch:37-50` | No for computation; always launch-started | ROS topics/TF from RViz config | visualization only | `far_planner/rviz/default.rviz` | None |
| `loamInterfaceTransPubMap` | `static_transform_publisher` / external `tf2_ros` | system launch `:52` | Intended TF bridge | none | `/tf_static`: `map -> camera_init` | positional launch args | Direction confirmed by parent/child argument order |
| `loamInterfaceTransPubVehicle` | `static_transform_publisher` / external `tf2_ros` | system launch `:54` | Intended TF bridge | none | `/tf_static`: `aft_mapped -> sensor` | positional launch args | Direction confirmed |
| `vehicleTransPublisher` | `static_transform_publisher` / external `tf2_ros` | `local_planner.launch` | Intended TF bridge | none | `/tf_static`: `sensor -> vehicle` | sensor offset launch args | Translation is negated offset |
| `sensorTransPublisher` | `static_transform_publisher` / external `tf2_ros` | `local_planner.launch` | Intended camera TF | none | `/tf_static`: `sensor -> camera` | camera offset + fixed rotation | Current camera consumer in mainline unconfirmed |
| `go2_safety_gate` | `go2_safety_gate` / `go2_keyboard_teleop` | `nav_launch.sh:131-137` | Yes for controlled output | `/cmd_vel_raw`, `/lf/sportmodestate`, `/arm`, `/stop`, lease response | `/api/sport/request`, lease request | source defaults; CLI dry-run/max speed/max yaw | Topic names are parameters, no launch remap |
| `wp5_explore_node` | `wp5_explore_node` / `go2_keyboard_teleop` | `nav_launch.sh:151-156` | Yes for autonomous goal generation | `/terrain_map`, `/state_estimation`, `/far_reach_goal_status` | `/goal_point` | source defaults; CLI arrive/timeout | No remap |

## Topic Inventory

### Critical mainline topics

| Effective topic | Message type | Publisher | Subscriber(s) | Source QoS | Remap | External | Hardware-related |
|---|---|---|---|---|---|---|---|
| `/utlidar/cloud` | `sensor_msgs/msg/PointCloud2` | GO2 onboard DDS | `transform_everything` | Subscriber: `sensor_data` | No | Yes | Yes, UTLiDAR |
| `/utlidar/imu` | `sensor_msgs/msg/Imu` | GO2 onboard DDS | `transform_everything` | Subscriber: `sensor_data` | No | Yes | Yes, onboard IMU |
| `/utlidar/transformed_cloud` | `sensor_msgs/msg/PointCloud2` | `transform_everything` | `laserMapping` | Publisher depth 50 default; subscriber depth 2000 default | Selected by Point-LIO YAML, not ROS remap | No | Derived sensor data |
| `/utlidar/transformed_raw_imu` | `sensor_msgs/msg/Imu` | `transform_everything` | No current mainline subscriber found | Publisher depth 50 default | No | No | Derived sensor data |
| `/utlidar/transformed_imu` | `sensor_msgs/msg/Imu` | `transform_everything` | `laserMapping` | Publisher depth 50 default; subscriber depth 2000 default | Selected by YAML | No | Derived sensor data |
| `/registered_scan` | `sensor_msgs/msg/PointCloud2` | `laserMapping` | terrain, terrain ext, local planner, FAR (remapped local terrain input) | Point-LIO pub depth 1000 default; consumers depth 1/5 default | From `/cloud_registered`; FAR `/terrain_local_cloud -> /registered_scan` | No | Derived sensor data |
| `/state_estimation` | `nav_msgs/msg/Odometry` | `laserMapping` | base odom, terrain, terrain ext, WP5, local planner, path follower | Point-LIO pub depth 1000; consumers depth 5/10 default | From `/aft_mapped_to_init` | No | Derived localization |
| `/base_state_estimation` | `nav_msgs/msg/Odometry` | `base_odom_node` | FAR | depth 10 default; FAR depth 5 default | FAR `/odom_world -> /base_state_estimation` | No | Derived localization |
| `/terrain_map` | `sensor_msgs/msg/PointCloud2` | `terrainAnalysis` | terrain ext, WP5, local planner, FAR | pub depth 2; subscribers depth 2/5/10 or FAR depth 5, default QoS | FAR `/scan_cloud -> /terrain_map` | No | Derived terrain |
| `/terrain_map_ext` | `sensor_msgs/msg/PointCloud2` | `terrainAnalysisExt` | FAR | pub depth 2; FAR depth 1, default QoS | FAR `/terrain_cloud -> /terrain_map_ext` | No | Derived terrain |
| `/goal_point` | `geometry_msgs/msg/PointStamped` | `wp5_explore_node`; optional RViz goal tool may also publish outside main script | FAR | WP5 depth 10; FAR depth 1, default QoS | No | Potential operator input | Indirectly controls motion |
| `/far_reach_goal_status` | `std_msgs/msg/Bool` | FAR | WP5 | both depth 5/10 default | No | No | Indirectly controls reselection |
| `/way_point` | `geometry_msgs/msg/PointStamped` | FAR | `localPlanner` | both depth 5 default | No | No | Planning command |
| `/navigation_boundary` | `geometry_msgs/msg/PolygonStamped` | FAR | `localPlanner` | both depth 5 default | No | No | Planning constraint |
| `/path` | `nav_msgs/msg/Path` | `localPlanner`; Point-LIO also creates a publisher but current YAML has `path_en=false` | `pathFollower` | local pub/sub depth 5; Point-LIO publisher depth 1000 | No | No | Motion-planning input |
| `/cmd_vel_raw` | `geometry_msgs/msg/TwistStamped` | `pathFollower` | `go2_safety_gate` | depth 5 -> depth 10 default | No | No | Pre-gate motion command |
| `/api/sport/request` | `unitree_api/msg/Request` | `go2_safety_gate`; shell stop helpers can publish one-shot | GO2 onboard DDS | gate depth 10 default; external subscriber QoS unknown | Parameter default, no remap | Subscriber external | Yes, real motion/stop API |

### Safety, control and graph support topics

| Topic | Type | Publisher -> subscriber | Notes |
|---|---|---|---|
| `/lf/sportmodestate` | `unitree_go/msg/SportModeState` | GO2 -> safety gate | External hardware state; gate depth 10 default. |
| `/arm` | `std_msgs/msg/Bool` | shell/operator -> safety gate | `nav_launch.sh` may publish automatically unless `--no-arm`. |
| `/stop` | `std_msgs/msg/Int8` | operator -> path follower and safety gate | Both subscribers use `Int8`; this is the current source contract. |
| `/api/sport_lease/request` | `unitree_api/msg/Request` | safety gate -> GO2 | Lease apply/renew APIs 101/102. |
| `/api/sport_lease/response` | `unitree_api/msg/Response` | GO2 -> safety gate | External QoS unknown. |
| `/update_visibility_graph` | `std_msgs/msg/Bool` | shell/operator -> FAR | One-shot published by `nav_launch.sh`. |
| `/robot_vgraph` | `visibility_graph_msg/msg/Graph` | FAR -> graph decoder | Both source endpoints use this absolute name. Decoder launch's `/planner_nav_graph` remap is ineffective because the executable does not use that name. |
| `/decoded_vgraph` | `visibility_graph_msg/msg/Graph` | graph decoder -> FAR | Relative name in decoder resolves to `/decoded_vgraph` in root namespace. |
| `/joy` | `sensor_msgs/msg/Joy` | external `joy_node` -> terrain/FAR/local/path follower | Hardware joystick configured at `/dev/input/js0`; autonomy behavior without device is runtime-dependent. |

## TF / Frame Inventory

| Parent / message frame | Child | Kind | Publisher/source | Consumer | Direction certainty |
|---|---|---|---|---|---|
| `body` | `utlidar_lidar_1` | Local `TransformStamped` object only | `transform_everything.py:74-85` | Used internally for point rotation | Names/direction explicit, but **not broadcast to `/tf`** |
| `body` | `utlidar_imu_1` | Local `TransformStamped` object only | `transform_everything.py:87-98` | Used internally for IMU rotation | Names/direction explicit, but **not broadcast to `/tf`** |
| `body` | — | Message frame | transformed cloud and IMU | Point-LIO | Explicitly assigned by transform node |
| `camera_init` | `aft_mapped` | Dynamic TF and odometry frame pair | Point-LIO `laserMapping.cpp:776-811` | TF consumers, downstream odometry | Confirmed by source |
| `camera_init` | — | Message frame | Point-LIO registered/map clouds and odometry header | terrain/local/FAR | Confirmed by source |
| `map` | `camera_init` | Static identity TF | `loamInterfaceTransPubMap` | FAR/RViz/TF users | Confirmed positional launch direction |
| `aft_mapped` | `sensor` | Static identity TF | `loamInterfaceTransPubVehicle` | TF users | Confirmed positional launch direction |
| `sensor` | `vehicle` | Static TF `(-sensorOffsetX, -sensorOffsetY, 0)` | `vehicleTransPublisher` | local/RViz/TF users | Confirmed by launch arguments |
| `sensor` | `camera` | Static TF with configured z offset and fixed rotation | `sensorTransPublisher` | camera/RViz users | Confirmed; current mainline consumer unknown |
| `map` | — | Message frame | terrain maps, WP5 goals, FAR outputs/debug | planners and RViz | Explicit in source/config |
| `vehicle` | — | Message frame | local planner `/path`, `/free_paths`; path follower `/cmd_vel_raw` | path follower/safety gate | Explicit in source |

The intended static/dynamic chain is `map -> camera_init -> aft_mapped -> sensor -> vehicle`, with an additional `sensor -> camera`. The transformed UTLiDAR messages use frame `body`, but there is no TF broadcast connecting `body` into this tree in `transform_sensors`. Point-LIO consumes the data using internal extrinsics, so a TF connection may not be required for Point-LIO itself; its system-wide necessity is `TODO: runtime verification required`.

Complete TF tree, timestamps, duplicate authorities and frame connectivity: **TODO: runtime verification required**.

## Main Data Flow Verification

| Segment | Result | Static evidence | Qualification |
|---|---|---|---|
| 1. UTLiDAR -> `transform_sensors` | **CONFIRMED BY SOURCE** | `transform_everything.py:26-30` subscribes to the two native topics with sensor-data QoS; preflight checks exact types | GO2 publisher and live samples are external/runtime facts |
| 2. `transform_sensors` -> Point-LIO | **CONFIRMED BY SOURCE** | transformer publishes the configured topics; `utlidar.yaml:11-13`; Point-LIO subscriptions in `laserMapping.cpp:915-917` | Runtime timestamp synchronization and rates unverified |
| 3. Point-LIO -> terrain | **CONFIRMED BY SOURCE** | launch remaps registered cloud/odometry; both terrain nodes subscribe to effective names | Point-LIO uses simulated time while terrain nodes default to wall time; runtime behavior unverified |
| 4. terrain -> WP5 / FAR / local planner | **CONFIRMED BY SOURCE** | `/terrain_map` subscribers in WP5/local/terrain ext; FAR launch remaps both terrain inputs | Terrain intensity semantic compatibility only partially documented |
| 5. WP5 -> FAR | **CONFIRMED BY SOURCE** | WP5 publishes and FAR subscribes `/goal_point` with matching `PointStamped` | Goal frame is `map`; actual receipt and cadence unverified |
| 6. FAR -> local planner | **CONFIRMED BY SOURCE** | FAR publishes and local planner subscribes `/way_point` with matching `PointStamped`; boundary also matches | Planner success/reach behavior unverified |
| 7. local planner -> pathFollower | **CONFIRMED BY SOURCE** | matching `/path` `nav_msgs/Path` pub/sub | Potential second Point-LIO publisher exists but is disabled by current YAML |
| 8. pathFollower -> safety gate | **CONFIRMED BY SOURCE** | matching `/cmd_vel_raw` `TwistStamped`; direct Sport publication in path follower is commented out | Runtime publisher exclusivity unverified |
| 9. safety gate -> GO2 Sport API | **PARTIALLY CONFIRMED** | safety gate publishes `unitree_api/Request` to `/api/sport/request`, implements lease/arm/fault logic | GO2 subscriber, lease acceptance and physical execution are external; `NOT VERIFIED` |

No segment in the documented main chain was statically contradicted. Segment 9 cannot be fully confirmed from this repository alone.

## Interface Inconsistencies

These are findings, not fixes.

1. **Mixed time sources in one launch tree.** `point_lio_unilidar/config/utlidar.yaml:3` sets `use_sim_time: true`, while `far_planner/launch/far_planner.launch:13` sets `use_sim_time=false`; other mainline nodes do not set it and therefore normally use wall time. No `/clock` publisher is started by the real-system launch. Severity: high; runtime effect needs verification.
2. **Point-LIO IMU-mode override conflict.** YAML says `use_imu_as_input: true` (`utlidar.yaml:5`) and `mapping/imu_en: true`, while `mapping_utlidar.launch:10` overrides top-level `use_imu_as_input` to `false`. Effective ROS parameter should follow the launch override, but the intended mode is not documented.
3. **Transform names without TF publication.** `transform_everything.py` assigns parent/child names for `body -> utlidar_lidar_1` and `body -> utlidar_imu_1`, but creates no broadcaster and never calls `sendTransform`. These objects are internal rotation containers, not TF tree edges.
4. **Potential `/path` name collision.** Point-LIO always creates a `/path` publisher (`laserMapping.cpp:929`), and local planner publishes the navigation `/path` (`localPlanner.cpp:608`). Current Point-LIO YAML disables path output (`utlidar.yaml:65`), avoiding data publication in the documented baseline. Enabling Point-LIO path would create same-name/same-type but different-semantics publishers.
5. **Incomplete `go2_keyboard_teleop` manifest.** Its Python nodes import/use `nav_msgs`, `std_msgs`, `unitree_go`, `sensor_msgs_py`, NumPy and related runtime packages, but `package.xml` declares only `rclpy`, `geometry_msgs`, `sensor_msgs`, `unitree_api` and an old `go2_robot_sdk` exec dependency. A clean rosdep environment may be incomplete.
6. **Incomplete `transform_sensors` dependency declarations.** Source imports NumPy, PyYAML and `transforms3d`; `setup.py` lists only `setuptools`, and `package.xml` does not declare all of these Python runtime dependencies.
7. **Current and legacy GO2 control flows coexist.** `go2_keyboard_teleop/package.xml` still declares `go2_robot_sdk`, and package docs/launch describe `cmd_vel_out`/WebRTC, while the current navigation path uses native DDS plus `/api/sport/request`. The current `nav_launch.sh` does not invoke the old driver path.
8. **Hard-coded legacy filesystem paths.** Multiple scripts still source or write `/home/lch/dog/...`; the current main scripts use repository-relative paths. Legacy scripts can fail or target the wrong workspace if treated as current entry points.
9. **Node-name duality.** Several source constructors use names different from launch overrides: `sensor_transformer` vs `transform_everything`, `far_planner_node` vs `far_planner`, and `graph_decoder_node` vs `graph_decoder`. ROS launch normally applies the requested runtime name, but logs and direct `ros2 run` behavior differ.
10. **QoS is explicit only at the native sensor boundary.** Native UTLiDAR subscribers use `qos_profile_sensor_data`; most internal publishers/subscribers pass only a depth and therefore use default reliable/volatile QoS. External GO2 state, Sport API and lease endpoint QoS cannot be confirmed from this repository. Runtime compatibility must be checked with endpoint introspection.
11. **`/stop` is shared by two consumers and uses `Int8`.** Both path follower and safety gate subscribe to `std_msgs/msg/Int8`, while any older documentation or operator command using `Bool` would be incompatible. Current `nav_launch.sh` prints the correct `Int8` command.
12. **Main launch starts joystick and RViz unconditionally.** `/dev/input/js0` and graphical RViz are launch-time dependencies even though they are not obviously required for the autonomous computation chain. Headless/no-joystick behavior is `TODO: runtime verification required`.
13. **Graph decoder launch contains a stale/ineffective remap.** `decoder.launch:17-32` declares and remaps `/planner_nav_graph`, but `decoder_node.cpp:21-24` subscribes to absolute `/robot_vgraph`. The current graph exchange still closes because FAR publishes `/robot_vgraph`; changing the launch argument has no effect on the actual subscription.
14. **Offline audit's RViz disable argument is not connected.** `offline_full_chain_audit.sh:25` passes `rviz:=false` to `system_real_robot_with_route_planner.launch`, but that top-level launch declares no `rviz` argument; it already forces Point-LIO RViz off while FAR launch starts `far_rviz` unconditionally. The audit's argument therefore does not disable FAR RViz and may be reported as unused by the launch frontend.
15. **Multi-terminal helper changes the reviewed control envelope.** `start_multi_terminal.sh:12-19` starts `nav_launch.sh --real --no-arm` and overrides `GATE_MAX_YAWRATE=0.50`, while the main script default is `0.30`. It is an explicit real-mode alternate entry point, not an observational monitor, and should not be treated as equivalent to the documented dry-run baseline.

## Runtime Verification Needed

- Run `ros2 node list` and `ros2 node info` for every launch-started node to confirm effective names and resolved remaps.
- Run `ros2 topic info -v` for every critical topic to capture actual publishers, subscribers, types and offered/requested QoS.
- Capture `ros2 param get` for `use_sim_time`, Point-LIO IMU flags and key node parameter sources.
- Generate a runtime TF graph and inspect `/tf` and `/tf_static` authorities, timestamps and connectivity.
- Confirm whether missing `/clock` blocks or skews Point-LIO in the real-system launch.
- Confirm whether `joy_node` exits when `/dev/input/js0` is absent and whether this affects other launch processes.
- Confirm headless offline launch behavior because the current audit script does not actually disable FAR RViz.
- Confirm graph decoder's `/robot_vgraph`/`/decoded_vgraph` endpoints and verify that the ineffective `graph_topic` launch argument is not relied upon operationally.
- Confirm that safety gate is the only continuous `/api/sport/request` publisher during the navigation run; shell stop publishers are expected to be transient.
- Verify GO2 external endpoint QoS and actual Sport lease/Request subscriptions without sending motion.
- Build from a clean dependency environment to validate package manifests and rosdep completeness.
