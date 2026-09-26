# 2026-09-01 完整包(数据+代码)

## 内容
- rosbags/ : 5 个代表实机复测 bag(retest4/5/6/8/9, 每轮最新)
- code/    : 当前源码(已回退到 8月31 打包版 code_package_20260831)
  - go2_keyboard_teleop/  (WP5探索/安全门/机身中心里程计)
  - autonomy_stack_go2_src/ (Point-LIO + terrain + far/local planner)
  - scripts/ (启动/录制/分析脚本)

## GO2 机载雷达接入

GO2 机载 UTLidar 的 `/utlidar/cloud` 和 `/utlidar/imu` 由机器人直接通过
ROS 2/CycloneDDS 发布，电脑端不需要 `go2_robot_sdk` 或额外启动雷达驱动。
将电脑有线网卡设置到 `192.168.123.0/24` 后执行：

```bash
cd /home/czx666/today_full_20260901/today_full
bash code/scripts/go2_lidar_preflight.sh <连接GO2的网卡名>
```

只有 cloud 与 IMU 的类型正确且收到实时数据时检查才通过。主导航脚本也会执行
同一项严格检查，雷达断流时直接退出，不再让 Point-LIO 空转。

## 代码状态(回退版)
- far: robot_dim=0.7, converge=0.30, graph去重
- local: obstacleInflate=0.2, stopDisThre=0.2, useCost=false
- WP5: min_goal_dis=0.8, arrive_dis=0.35, 簇门槛2, 超时硬排除, risk_weight=0.3, min_obs_dis=0.2
- 注: 无 turn_thre / 走廊检查 / 方向惯性 / 净空惩罚(这些是今天后加又回退的)

## 代表 run 结论
- retest4(78s): 能走0.82m但94%时间在转
- retest5(258s): 今天最远2.96m, 仍67%转
- retest6(219s): 走廊检查误杀→几乎不动
- retest8/9: 回退后, 能走一点

详见 today_representative.md(在 analysis 包 today_analysis_20260901.tar.gz)

## Project Management

- `AGENTS.md`：ChatGPT 网页版与 Codex 的长期协作规则、安全边界和验证要求。
- `PROJECT_STATUS.md`：当前可确认的系统状态、基线、问题和待验证项。
- `TASKS.md`：当前阶段的项目接管与可复现性任务清单。
- `docs/`：现有架构、设计决策和实验记录。
