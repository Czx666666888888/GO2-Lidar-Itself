# Tasks

当前阶段只做项目接管和基线确认。完成标准是形成可审查的清单与验证证据，不包含新算法开发。

## P0 — Baseline Understanding

- [x] 建立完整 ROS 2 package 与主链 node 静态清单，并标记主线/辅助/仿真/旧或替代流程（见 `docs/interface_inventory.md`；运行时名称仍待验证）。
- [ ] 建立 topic、消息类型、QoS、publisher/subscriber 与 data-flow 清单。
- [ ] 建立 TF / coordinate frame 清单，并用运行时 TF tree 验证静态结论。
- [ ] 确认 GO2 UTLiDAR 数据入口、DDS 前提与无 PC 侧 driver 的边界。
- [ ] 确认 Point-LIO 配置入口、时间源、输入输出和标定依赖。
- [ ] 确认 terrain analysis 与 terrain analysis ext 的输入输出及职责差异。
- [ ] 确认 FAR planner 的入口、配置、visibility graph 和 waypoint 输出。
- [ ] 确认 local planner/path follower 的入口、路径数据与速度输出。
- [ ] 确认 waypoint / WP5 exploration 的候选、到达、超时和重选流程。
- [ ] 确认 `/cmd_vel_raw` 经 safety gate 转为 GO2 Sport 请求的完整路径。
- [ ] 核对 README、launch、源码和脚本之间的接口/参数不一致项。

## P1 — Reproducibility

- [ ] 在干净 Ubuntu 22.04 / ROS 2 Humble 环境确认依赖列表。
- [ ] 确认唯一推荐的 build 命令，并记录 package 级构建结果。
- [ ] 确认主线启动、dry-run、停止和残留进程检查方法。
- [ ] 将测试分类为：无 ROS、ROS 离线、合成数据、依赖 rosbag、依赖 GO2 真机。
- [ ] 运行不依赖真机的 lint/unit/synthetic 测试并记录结果。
- [ ] 选择一组本地 rosbag，记录来源、校验值与离线回放方法。
- [ ] 验证离线链的关键 topic、频率和消息类型，不发送硬件命令。
- [ ] 建立 GO2 真机前置条件和分阶段安全验证表。

## Out of Scope for This Phase

- 新增或替换规划算法
- 调参以改善 retest 结果
- 修改 Point-LIO、WP5、FAR/local planner 或 safety gate 行为
- 未经评审的 ROS interface 或架构重构

## D435 Standalone Perception Workspace

- [x] 创建独立 `camera_ws` 和 ROS 2 Humble Python package。
- [x] 增加 RGB/aligned-depth/CameraInfo 只读诊断节点及 CameraInfo 内参检查节点。
- [x] 增加集中式 RealSense wrapper 配置和独立 camera test launch。
- [x] 记录 pixel、camera optical、GO2 body、map 坐标边界与未来标定要求。
- [x] 只读确认 D435 USB/UVC 枚举状态并记录 SDK/wrapper 缺失。
- [x] 经用户授权后安装官方 RealSense ROS 2 Humble packages，并记录精确版本与来源。
- [x] 实机确认 RGB、raw depth、aligned depth、CameraInfo 与相机内部 frame/TF 数据可用。
- [x] 增加 RGB/aligned-depth OpenCV 查看器及 5×5 ROI median 米制深度探针。
- [x] 增加独立蓝色目标上表面多平面 RANSAC 中心节点、CameraInfo 反投影、debug 窗口和 camera-frame PointStamped。
- [x] 将蓝色上表面链路改为形态学稳定 mask、上部2D ROI和单主平面拟合，并加入连续 VALID/丢失统计。
- [x] 增加逐帧mask/ROI/depth/RANSAC/法向指标、7类INVALID计数和30秒汇总。
- [x] 回退上部55% ROI，恢复完整target mask最多4平面RANSAC，并实现先35°法向过滤、再按内点数/残差选择。
- [x] 增加mask有效深度中位数粗距离、分段RANSAC阈值、动态最少内点和三级感知质量；完成近/中/远三档各15秒受控实测，质量不足时不发布精确3D中心。
- [x] 保留precise/backup节点不变，新增独立HSV mask质心+median depth粗定位节点和 `/science/target_coarse_point`；完成0.5/1.0/1.5/2.0 m四档实机记录。
- [x] 将粗定位扩展为多蓝色目标与3D蓝墙过滤；完成仅蓝墙、蓝墙+1目标、蓝墙+3个不同大小目标的静态实机验证，同一帧沿既有topic连续发布每目标PointStamped。
- [x] 将蓝墙剔除从仅RANSAC inlier扩展为参数化全平面距离带，并增加cluster墙距
  median/min/max诊断和门控；固定3目标+1蓝墙现场两个独立35秒窗口均严格每帧3点，
  manager confirmed稳定为3个ID。近共面目标仍需单独验证。
- [x] 新增参数化 `vehicle -> camera_link` 暂定外参、按原PointStamped时间戳转换到map的节点和RViz Marker；仅启动SLAM+D435+RViz完成一次静止目标/人工移动GO2 A/B验证（均值差约14.9 mm），未接FAR/WP5。
- [ ] 使用外部测量真值完成camera外参正式标定，并重复纯平移、纯旋转、多距离/方位及SLAM长期漂移试验；当前外参仅为 `PROVISIONAL / NOT CALIBRATED`。
- [ ] 多平面版本30秒实测VALID 56/204（27.5%），主要失败仍为 `bad_normal=118`；需在固定目标/固定相机/固定光照条件下采集受控对比数据后再决定优化方向。
- [ ] 在可交互桌面对近处/远处目标各执行一次人工点击并补录深度值；当前 `NOT VERIFIED`。
- [ ] 排查本机 Python 大图像订阅吞吐（实测低于 30 Hz且有短暂 aligned-depth freshness 告警），完成持续速率验证。
- [ ] map转换已完成只读原型验证；未来仅在外参正式标定和重复稳定性验证通过后，另行评审是否接入导航。
- [x] 新增不接FAR、不运动的`science_target_manager`第一阶段：同帧多点聚类去重、
  5帧确认、近期median、稳定ID、全部CONFIRMED MarkerArray、最近未访问目标和0.4 m
  standoff可视化；机身中心严格按`/base_state_estimation`实际frame经TF转到map。
- [x] 在只读真机组合中验证`/base_state_estimation`实际frame、到map的连续TF、同帧
  多目标Marker及RViz显示；2026-10-03已确认`camera_init -> map`转换、多个confirmed、
  selected/standoff及visited重选，且未连接`/goal_point`。现场候选累计到至少T18，
  每个ID是否对应独立真实目标仍需受控真值实验。
- [x] 完成3个固定真实目标、GO2前后/左右/原地左右转/斜移的map稳定性实验：原始
  T1–T3最大跨阶段均值XY差34.62 mm，但旋转/斜移后最终增长为7个confirmed ID，
  selected误选额外track T14；详细统计见`camera_ws/docs/experiments.md`。
- [x] 不改前端或0.20 m关联半径，为manager增加candidate/confirmed/stale老化、删除、
  连续多帧近邻合并及active-confirmed-only选择；2026-10-06三固定目标8阶段重跑的每个
  稳定窗口均收敛为3个ACTIVE CONFIRMED，standoff为0.400 m。实机未触发MERGED，
  合并实机触发仍为`NOT VERIFIED`，且54.02–64.28 mm视角偏差/暂定外参仍禁止FAR接入。
- [x] 将访问状态改为不依赖track ID的map空间记忆：默认0.15 m半径、Empty触发完成当前
  selected、visited/ignored与历史点RViz marker、空间过滤及兼容旧ID入口；54项package
  tests和隔离build通过。
- [x] 用3个固定蓝色目标完成visited空间记忆真机验收：2026-11-03人工触发后eligible
  `3 -> 2 -> 1 -> 0`；遮挡原T2后重新出现为T70，仍以map XY距离0.028 m命中旧visited
  位置并保持排除。遮挡动作曾造成短暂蓝色误检，恢复稳定12 s后selected持续为空。
- [ ] FAR science goal dry-run：已增加一次性`science_goal_bridge`并完成56项package tests、
  独立build和接口审计；现场启动污染轮曾把`(0.507,0.049,0.021)`发给FAR，但该轮因
  启动误检作废。干净重试又确认额外T6并导致selected切换，按安全门槛停止，尚未获得
  有效科学目标的`/goal_point -> /way_point -> /path -> /cmd_vel_raw`完整证据。

## 2026-11-06 — 用户授权的独立单目标接近实验

本段扩展既有阶段范围，不修改Point-LIO/FAR/WP5或蓝色识别算法，不授权真实运动。

- [x] 复用manager ID/visited及原观测时间，增加人工确认、锁定后不切换的独立控制器。
- [x] 新增独立参数、仅预览launch、专用gate配置（min_speed=0、max_speed=0.30）。
- [x] 增加带真实姿态TF的机身中心几何、窗口位姿差分速度/ETA、制动限速和故障锁存。
- [x] GO2 Foxy独立构建相机驱动，用户完成对时，PC连续收到远程RGB/对齐深度。
- [x] 用户调参后静态dry-run单目标稳定，T1锁定及视觉/定位输入超时归零、锁存通过。
- [ ] 第一阶段完整验收：人工动态dry-run未完成时，用户明确改为直接做受限实机；
  后续完整实机轮视觉丢失提前停车，仍未完成可见目标0.80 m定距停车。
- [x] 用户将源码默认s_min调至110、面积调至250；同步运行YAML并实收两窗均每帧1目标。
- [x] 用户明确确认现场模式/控制权、平整无障碍、急停及监护，并授权0.10 m/s短时测试；
  命令时间截止停车，用户确认前进了一点。随后另授权0.30 m/s上限完整接近，
  用户确认方向正确且视觉丢失后已停住、无明显异常。
- [ ] 实测净空、外部速度/位置真值、连续减速及0.80 m稳定停车精度；当前均不可据命令
  限速或一次故障停车认定通过。完整轮实测位置差分峰值0.494 m/s需先核验。
- [ ] 阶段二停车及故障停止实机验收后另行启用有时间/实测路径上限的APPROACH_ODOM；
  当前真实模式仍禁止启用。

## 858f75a后续 — 视觉/MAP锚点与相机原点对齐

- [x] 分析旧视觉丢失日志，发现输出间隔/新鲜度门槛问题，未大幅重新调HSV。
- [x] 稳定中位数锚点、ID刷新/视觉间断保持及空间匹配平滑修正。
- [x] 相机原点带时间戳TF、XY 0.10 m停车、有时限/路径上限的真实ODOM闭环。
- [x] 实际RGB+原检测轮廓/中心/mask/状态MP4，同名逐帧/控制JSONL及完整解码。
- [x] 现场定位发散/时钟问题在运动前拦截并重建只读链，不修改Point-LIO核心。
- [x] 记录kp=0.5的35cm超时失败，kp=1.0的约10cm用户确认停车成功。
- [x] 提供显式执行一键入口和独立实时日志终端，不新增运动接口。
- [ ] 不同初始距离/姿态/光照下重复精度、正式相机外参、实际速度严格上限定量验收。
