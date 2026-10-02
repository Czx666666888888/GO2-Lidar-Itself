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
- [x] 新增参数化 `vehicle -> camera_link` 暂定外参、按原PointStamped时间戳转换到map的节点和RViz Marker；仅启动SLAM+D435+RViz完成一次静止目标/人工移动GO2 A/B验证（均值差约14.9 mm），未接FAR/WP5。
- [ ] 使用外部测量真值完成camera外参正式标定，并重复纯平移、纯旋转、多距离/方位及SLAM长期漂移试验；当前外参仅为 `PROVISIONAL / NOT CALIBRATED`。
- [ ] 多平面版本30秒实测VALID 56/204（27.5%），主要失败仍为 `bad_normal=118`；需在固定目标/固定相机/固定光照条件下采集受控对比数据后再决定优化方向。
- [ ] 在可交互桌面对近处/远处目标各执行一次人工点击并补录深度值；当前 `NOT VERIFIED`。
- [ ] 排查本机 Python 大图像订阅吞吐（实测低于 30 Hz且有短暂 aligned-depth freshness 告警），完成持续速率验证。
- [ ] map转换已完成只读原型验证；未来仅在外参正式标定和重复稳定性验证通过后，另行评审是否接入导航。
