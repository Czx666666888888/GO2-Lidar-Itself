# Project Status

更新日期：2026-09-26
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
- `utlidar.yaml` 设置 `use_sim_time: true`；实时链是否提供 `/clock` 需要验证。
- `transform_sensors` 从用户 Desktop 读取 `imu_calib_data.yaml`，文件缺失时使用源码默认值；当前真实标定来源和有效性尚未确认。
- 仓库包含旧流程、当前 DDS 流程、仿真和实验脚本，哪些仍是受支持入口尚未形成正式清单。

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
- 24 个 package 中哪些属于当前主线、辅助工具、旧流程或未使用模块？
- 实时运行时 `/clock` 是否存在，Point-LIO 的 `use_sim_time` 是否与主线一致？
- GO2 当前固件、ROS domain、CycloneDDS QoS 和网卡配置的正式支持矩阵是什么？
- `imu_calib_data.yaml` 的权威版本、生成方法和部署位置是什么？
- 主链真实 TF 树及各点云/里程计 frame 是否一致？
- README 的 retest 指标如何计算，分析脚本与原始日志能否复现这些数值？
- safety gate 的 Sport lease、ARMED 和 API 成功能否对应到可测量的真实运动？

## Next Recommended Work

仅建议调查，不在本轮实施：

1. 在无真机条件下建立 package、node、topic、frame 和参数的完整静态清单。
2. 在干净 ROS 2 Humble 环境执行依赖检查与可重复 build，记录精确命令和失败项。
3. 分类现有测试：纯静态/单元、合成数据、依赖 rosbag、依赖 GO2 真机。
4. 使用一组明确来源的 rosbag 做只读离线回放，验证 Point-LIO 到 `/cmd_vel_raw`，不启动真实运动输出。
5. 完成硬件安全清单后，再设计分阶段 dry-run 和低速真机验证。
