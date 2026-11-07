# D435 Perception Architecture

## Isolation Boundary

`camera_ws` 是独立 ROS 2 Humble 工作空间。本阶段不引用、不启动也不修改现有 Point-LIO、terrain、WP5/NBV、FAR、local planner、path follower、safety gate、GO2 Sport API 或主导航 launch。

## Planned Data Chain

```text
RealSense D435
  -> RGB image
  -> depth image
  -> color/depth CameraInfo
  -> aligned depth to color
  -> Blue HSV object mask                   [IMPLEMENTED: isolated test node]
  -> morphology-cleaned main component      [IMPLEMENTED]
  -> aligned-depth pixels in mask           [IMPLEMENTED]
  -> CameraInfo intrinsics                  [runtime input]
  -> 3D position (Xc, Yc, Zc)
     in camera color optical frame          [IMPLEMENTED]
  -> complete target-mask local point cloud [IMPLEMENTED]
  -> mask valid-depth median coarse distance [IMPLEMENTED]
  -> piecewise distance-adaptive RANSAC threshold [IMPLEMENTED]
  -> dynamic min inliers from absolute/ratio floor [IMPLEMENTED]
  -> sequential RANSAC, up to four planes   [IMPLEMENTED]
  -> 35deg normal filter, then support/fit ranking [IMPLEMENTED]
  -> GOOD / MARGINAL / UNRELIABLE quality gate [IMPLEMENTED]
  -> robust top-surface center PointStamped [GOOD only]
  -> parameterized vehicle -> camera_link   [PROVISIONAL / NOT CALIBRATED]
  -> timestamped TF2 transform              [IMPLEMENTED]
  -> SLAM map PointStamped + Marker         [IMPLEMENTED / NOT NAVIGATION-CONNECTED]
  -> science_target_manager                 [IMPLEMENTED / VISUALIZATION ONLY]
     -> per-frame XY association + median tracks
     -> CANDIDATE -> CONFIRMED after 5 distinct frames
     -> stable target IDs + all-target MarkerArray
     -> nearest unvisited target + 0.4 m standoff visualization
```

当前实现还包括独立的蓝色上表面中心测试节点；它只输出相机 optical frame 坐标，不接入 GO2 或 map。

另有独立 `coarse_target_locator` 粗定位节点。它保留HSV与形态学作为全部蓝色候选，不再只取最大连通域；aligned-depth蓝色像素先反投影到3D，并通过RANSAC优先识别“大范围、近似竖直、平面支持度高”的蓝墙。蓝墙内点被标记为rejected，剩余蓝色区域先按2D连通性、再按深度间隔重新聚类；同一2D组件内低于15%支持度的小深度层作为深度噪声剔除，但不同2D组件不要求相同面积或固定物理尺寸。每个有效簇分别计算2D centroid、腐蚀mask内median depth和运行时CameraInfo反投影XYZ，同一帧N个目标向既有 `/science/target_coarse_point` 连续发布N个 `PointStamped`，frame与原始aligned-depth时间戳保持不变。该分支与precise算法并存，不改变`camera_target_to_map`。

距离自适应以完整目标mask内有效深度的中位数为粗距离。默认RANSAC距离门槛为：小于1.0 m使用0.008 m，1.0至1.5 m使用0.012 m，1.5 m及以上使用0.018 m。RANSAC最少内点为 `max(ransac_min_inliers_absolute, ceil(point_count * ransac_min_inliers_ratio))`。

`perception_quality` 同时检查 `mask_area`、`valid_depth_ratio`、`inlier_ratio` 和 `normal_error_deg`。四项均达到GOOD门槛才向 `/science/blue_surface_center` 发布精确3D中心；MARGINAL和UNRELIABLE仍保留mask、候选平面、质量和距离debug信息，但不发布中心。默认GOOD门槛为800 px、0.65、0.18、20°；MARGINAL门槛为200 px、0.35、0.08、35°。这些是首轮参数，需后续受控实验复核。

源码default与运行YAML中的 `mount_pitch_deg` 当前统一为30.0°。2026-09-29三档实验是在统一前使用YAML的45.0°完成，其结果不能直接代表当前30.0°配置。

## Current Nodes

| Node | Inputs | Outputs | Side effects |
|---|---|---|---|
| Official `realsense2_camera` wrapper | D435 USB streams | image, CameraInfo, camera-internal TF | Opens camera only |
| `camera_diagnostics` | RGB, aligned depth, color/depth CameraInfo | Log report | Read-only |
| `camera_info_inspector` | Color CameraInfo | Intrinsics log | Read-only |
| `blue_surface_center` | RGB, aligned depth, color CameraInfo | `/science/blue_surface_center`, debug window | Camera-frame perception only |
| `coarse_target_locator` | RGB, aligned depth, color CameraInfo | 每目标一个`/science/target_coarse_point`, wall/target/centroid debug | Multi-target coarse camera-frame perception only |
| `camera_target_to_map` | `/science/target_coarse_point`, TF | `/science/target_coarse_point_map`, Marker, `vehicle -> camera_link` static TF | Read-only map projection |
| `science_target_manager` | `/science/target_coarse_point_map`, `/base_state_estimation`, `/science/mark_selected_visited`, optional legacy `/science/visited_target_id` | confirmed/visited/selected/standoff Point/MarkerArray topics | Target bookkeeping and visualization only; never publishes `/goal_point` or motion commands |
| `science_goal_bridge` | `/science/standoff_goal` | `/goal_point` | Optional one-shot FAR dry-run adapter; locks the first finite `map` goal and ignores later selected changes |

The perception and manager nodes do not publish robot velocity, Sport API requests or
SLAM data. `science_goal_bridge` is the only optional navigation-goal publisher and is
not part of the normal D435 validation launch.

### Science target manager

`science_target_manager`把具有相同原始时间戳的连续
`/science/target_coarse_point_map`消息作为同一帧处理。每帧执行一对一最近邻关联，
默认只在map XY距离不超过`association_radius=0.20 m`时更新已有轨迹，且同一轨迹
在一帧内最多累计一次。新轨迹从`CANDIDATE`开始，在5个不同帧中累计关联后成为
`CONFIRMED`；位置取最近15次观测各坐标的median。每条track记录`first_seen`、
`last_seen`、`observation_count`和`CANDIDATE / CONFIRMED / STALE`状态。默认1 s未更新
的candidate删除，confirmed默认2 s未更新转为STALE、10 s未更新删除；STALE重新关联
后恢复CONFIRMED，但处于STALE期间不能被selected。ID从1单调分配。

两个active confirmed只有连续5帧的XY距离均小于`merge_radius=0.10 m`才合并；任一帧
不再满足就清零该pair计数。保留状态更稳定、观测更多、首次出现更早（最后以较小ID
打破平局）的track，合并两者当前观测窗口后重新计算median，并删除旧track。RViz
每次发布先对confirmed sphere和label两个namespace发送`DELETEALL`，因此老化或合并
掉的ID不会留下旧marker。该机制独立于`association_radius`，后者仍保持0.20 m。

输出接口如下：

- `/science/confirmed_targets` (`visualization_msgs/MarkerArray`)：同时显示全部已确认
  目标，sphere和文字标签均使用track ID，避免单一`marker id=0`覆盖。位于访问半径内
  的track显示为灰色`visited/ignored`；历史访问map点使用独立
  `science_visited_positions` namespace显示。
- `/science/selected_target` (`geometry_msgs/PointStamped`)及
  `/science/selected_target_marker`：只从active、`CONFIRMED`且未访问目标中选取map XY
  距机身中心最近者；STALE、已删除/合并以及与任一历史visited位置XY距离小于
  `visited_radius=0.15 m`的目标均不参与。track ID允许变化，排除依据是map空间位置。
- `/science/standoff_goal` (`geometry_msgs/PointStamped`)及
  `/science/standoff_goal_marker`：从目标朝GO2当前中心方向退0.40 m；z使用GO2当前
  map高度。正常感知launch中该点仅是可视化候选；仅在显式启动
  `science_goal_bridge`的FAR dry-run中，首个有效map-frame点会被一次性转发。
- `/science/mark_selected_visited` (`std_msgs/Empty`)：人工完成当前selected目标后触发；
  将该track当前map位置写入`visited_positions`，若与已有位置小于0.15 m则合并，随后
  立即重新选择。人工触发命令为：
  `ros2 topic pub --once /science/mark_selected_visited std_msgs/msg/Empty '{}'`。
- `/science/visited_target_id` (`std_msgs/Int32`)：兼容旧操作方式；收到active confirmed
  ID后同样把该track当前位置写入空间记忆，不再以ID visited flag作为主要过滤逻辑。

`visited_positions`是节点进程内空间记忆；manager重启后清空，当前未做磁盘持久化。
日志包含`VISITED_ADDED`、`VISITED_MATCH`、`VISITED_DUPLICATE`和
`SELECTED_CHANGED`，用于记录访问位置、新旧track命中距离及重选过程。

`/base_state_estimation`的源码事实是`base_odom_node`复制Point-LIO里程计header；
Point-LIO当前发布`header.frame_id=camera_init`，不是`map`。管理器首次收到里程计时
记录实际frame，并按消息时间戳通过TF转换到`map`；frame为空或TF不可用时拒绝更新
机身位置，禁止直接把不同frame的数值混用。当前D435/SLAM验证launch显式提供
`map -> camera_init`静态桥并启动`base_odom_node`。运行时实际frame与TF连续性仍需
实机确认。

安全边界：管理器没有`/goal_point` publisher，不启动FAR/WP5/local planner/path
follower/safety gate，也不发布速度或GO2 Sport请求。可选bridge只发布一次
`PointStamped /goal_point`，拒绝非map frame及非有限数值，并锁定首个目标；它不发布
`/way_point`、`/path`、`/cmd_vel_raw`或GO2 Sport请求。

## Coordinate Systems

### Pixel coordinates `(u,v)`

Two-dimensional image indices. `u` increases to the image right; `v` increases downward. A color pixel may index aligned depth only when depth has actually been aligned to the color stream.

### Camera optical coordinates `(Xc,Yc,Zc)`

Metric three-dimensional coordinates expressed in the optical frame named by the relevant message `header.frame_id`. ROS optical convention is normally `+X` right, `+Y` down, `+Z` forward. The runtime frame ID remains authoritative.

For a rectified pinhole image and depth `Zc`, both current perception nodes use runtime `CameraInfo` values:

```text
Xc = (u - cx) * Zc / fx
Yc = (v - cy) * Zc / fy
Zc = aligned depth converted to metres
```

`coarse_target_locator`的结果仍在源相机optical frame。独立 `camera_target_to_map` 节点通过TF2按该点原始时间戳转换到 `map`，并发布map点和Marker；该转换不改变粗定位节点。

当前暂定外参只定义在ROS机械frame之间，不直接对optical frame写RPY：

```text
map -> camera_init -> aft_mapped -> sensor -> vehicle
                                             -> camera_link
                                                -> camera_color_optical_frame
```

其中 `vehicle -> camera_link` 参数默认平移为 `(0.36, 0.00, 0.12) m`，RPY为 `(0, +30, 0) deg`；最后一段由RealSense wrapper已有TF提供。`camera_target_to_map`订阅 `/science/target_coarse_point`，发布 `/science/target_coarse_point_map` 和 `/science/target_coarse_point_map_marker`。这些外参是暂定值，不等于完成标定。

`camera_target_to_map`始终按输入PointStamped的原始相机时间戳查询TF，不允许用“最新TF”替代。若Point-LIO动态TF尚未覆盖该时间戳，节点记录point timestamp、buffer中latest TF timestamp和`point-latest_tf`时间差，并将原消息放入有界队列；默认每20 ms重试、最长8 s、最多128点。只有同一原始时间戳的TF随后可用才发布，超时、队列溢出或关闭时仍未完成的点计为failed。D435实机组合launch显式向Point-LIO传`use_sim_time=false`；Point-LIO原launch默认仍为`true`，保留rosbag/replay用途。

### GO2 body coordinates

Robot-fixed coordinates defined by the future mounting/extrinsic convention. They are not camera optical coordinates. The authoritative GO2 body/base frame and axis convention require an explicit integration decision and measured rigid transform.

### Map coordinates

World-fixed coordinates generated by the GO2 radar SLAM chain. A point must not be labelled as `map` merely because it has three numeric components. It requires a valid time-aware TF chain from the point's actual source frame.

## Frame Rules

- Never copy a point between frames by changing only `header.frame_id`.
- Every future `PointStamped` must retain the source image timestamp and accurate optical `frame_id`.
- Camera-to-body extrinsics and body-to-map localization are separate transforms.
- 2026-09-28 实测 `/tf_static` 相机内部链为：

```text
camera_link -> camera_depth_frame -> camera_depth_optical_frame
camera_link -> camera_color_frame -> camera_color_optical_frame
```

- RGB 和 aligned-depth 消息的实测 frame 均为 `camera_color_optical_frame`；raw depth 和 depth CameraInfo 为 `camera_depth_optical_frame`。
- 当前 `tf_publish_rate=0.0`，因此相机内部关系通过 `/tf_static` 发布，实测没有相机 wrapper 的动态 `/tf` 数据。
- D435 has no IMU; this architecture has no camera-IMU dependency.

## Future Integration Gate

Before any map publication or navigation consumption:

1. Verify synchronized RGB/aligned-depth samples and runtime CameraInfo.
2. Validate pixel-to-camera 3D geometry against measured targets.
3. Measure and document camera-to-GO2-body extrinsics.
4. Identify the authoritative GO2 body/base frame and continuous TF path to `map`.
5. Validate timestamps and transform availability under robot motion.
6. Only after review, define a target topic for navigation consumption.

第1、2、4、5项已在2026-09-29单次实机链路中进行初步验证；第3项仍只有暂定手量外参，第6项明确不在本轮范围。不得将这次约15 mm的单次A/B差值解释为外参标定完成或导航可用。

## 2026-11-06 — Optional independent approach experiment

用户另行授权新增不依赖FAR的 `science_target_approach`，不改变上述视觉数据流。
manager 新增 `/science/target_tracks` String JSON快照（frame、ID、state、position、
observed_ns、first_seen_ns、visited），原接口保持兼容。接近控制器消费该快照、
按消息时间经TF转换的 `/base_state_estimation` 完整pose、`/lf/sportmodestate`，
人工核对candidate后确认具体ID。默认只输出 `/science/approach/cmd_vel_preview` 和
`/science/approach/status`；真实模式的输出边界为 `/cmd_vel_raw -> go2_safety_gate -> Sport`，
节点本身不发布Sport请求、不武装。真实模式检查gate参数和唯一可发现本机Sport输出者。
FAR/WP5/localPlanner/pathFollower不参与此实验。阶段一因误选伪目标尚未通过；
用户后续授权两次实机，方向及视觉丢失后实际停车由用户确认；0.80 m定距停车与实际
速度上限仍NOT VERIFIED。远程相机Foxy配置、启动和验证边界见 `science_target_approach.md`。

## MAP锚点相机原点模式（858f75a后续）

接近节点现以短窗口中位数建立MAP锚点；锁定ID只作标识，后续观测按初始锚点与当前锚点
双重空间门槛关联并平滑修正。track删除/刷新不删除锚点。VISUAL使用新鲜观测修正，ODOM
持有锚点、仍依实时Point-LIO及map->camera_link TF闭环。终点为相机原点MAP XY距离≤0.10 m。
原body_center模式仅保留兼容。允许真实ODOM但有时间/实测路径边界，定位/TF异常仍停止。

coarse_target_locator新增只读`/science/detection_debug` String及`/science/detection_mask`
Image；算法不变。approach_video_recorder消费实际RGB和该检测结果及控制status，输出MP4+
JSONL并发布`/science/approach/recording`健康状态；控制器可要求录像健康。默认launch为
预览+录像，显式运行脚本才经原gate执行一次，并默认打开只读日志终端。见运行说明与实验记录。

## 2026-11-07 — 单控制器移动搜索

`science_target_manager`快照仅新增`latest_observation`（本帧未平滑MAP观测），
保留源`observed_ns`。`science_target_approach`的可选SEARCH使用最近1秒不同源帧做
MAP XY散布检验与中位数锚定，不使用像素静止判据；确认后零输出一周期再进入原接近
状态机。SEARCH按启动MAP朝向直行，默认0.20 m/s/1m/20s，有路线/横向边界和超时停止。
同一节点始终独占`/cmd_vel_raw -> go2_safety_gate -> Sport`，不引入第二个运动发布者。
视觉/SLAM/TF链及camera_link原点XY停车语义不变；独立SEARCH配置覆盖参数，人工接近
配置保留。原始视频/检测/控制日志共同记录SEARCH与APPROACH。实机结果与验证边界见
同目录experiments.md，源代码/参数存在不等于所有现场条件通过。
