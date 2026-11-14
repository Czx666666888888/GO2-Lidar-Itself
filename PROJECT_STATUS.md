# Project Status

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
