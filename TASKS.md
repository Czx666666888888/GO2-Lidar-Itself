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
