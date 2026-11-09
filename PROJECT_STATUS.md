# Project Status

更新日期：2026-10-06
状态来源：当前仓库静态阅读、根 `README.md` 的既有记录和 Git 历史。除非特别说明，本文件不代表本轮重新完成了真机构建或运行验证。

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

## Current Baseline

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
