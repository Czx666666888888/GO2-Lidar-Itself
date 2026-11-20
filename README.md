# GO2-Lidar-Itself

基于 ROS 2 的 Unitree GO2 自主导航与科学目标感知实验项目，包含 UTLiDAR/IMU、Point-LIO、FAR/局部规划、Sport 安全门、D435 RGB-D、MAP 停靠与 S1 抓取，以及独立 YOLO 数据和训练工程。

更新日期：**2026-11-20**。最新研发代码位于 [`feature/science-targets-perception`](https://github.com/Czx666666888888/GO2-Lidar-Itself/tree/feature/science-targets-perception)。当前事实及后续事项以 [PROJECT_STATUS.md](PROJECT_STATUS.md) 和 [TASKS.md](TASKS.md) 为准。

## 开始使用

PC 基础环境为 Ubuntu 22.04 / ROS 2 Humble。默认入口仅显示离线计划或运行离线检查；真实执行须由操作者确认环境和急停条件后主动启动。

```bash
# PC：查看原 MAP + S1 FULL 离线计划
bash go2_test.sh

# PC：最小规则避障的离线回归，不连接机器人
bash go2_test.sh --mode avoid --dry-run

# PC：独立导航的离线构建、回归和启动配置解析
bash go2_nav_test.sh --dry-run
```

- PC 搜索、停靠、抓取、洞口前置段及停止路径：[实机测试手册](GO2实机测试命令手册.md)。
- PC 独立雷达避障：[avoid_ws/README.md](avoid_ws/README.md)。
- PC 独立导航往返与 RViz2：[navigation_test/README.md](navigation_test/README.md)。
- NX 自主 FULL、可调计时前置速度、持久终端与断线重连：[最终操作命令](docs/go2_nx_final_commands.md)。这些命令依赖已部署的 NX 环境，clone 本仓库不能代替部署。

不要同时启动多个运动入口。真实运动须具备物理急停、空旷路线和可确认的停止路径；本次 GitHub 同步不执行机器人动作。

## 当前进展与验证边界

| 工作流 | 当前实现及已有证据 | 尚未验证或已知问题 |
|---|---|---|
| PC MAP + S1 | 一键检查、对时、相机/定位、SEARCH → STOP_CONFIRM → MAP 停靠 → S1；可选洞口段 | 完整多轮重复精度、轮间恢复及机械抓取可靠性仍 `NOT VERIFIED`；失败轮次见实验记录 |
| NX 自主 FULL | 持久任务终端、人工确认、可调计时直行、多目标；近场交接结束导航进程并隔离遗留导航回调，保留目标 ID 与抓取侧检查 | 最新修正后的接近、抓取、下一目标及返航，实际拔线和其他电脑登录均待现场验收 |
| 独立导航 | Point-LIO → 地形 → NBV/FAR → 局部规划/跟踪 → Sport gate；记录路径返航、RViz2、分区前停转向与侧移 | 已有离线及隔离 DDS 模拟记录；实机完整往返、物理净距和重复性能 `NOT VERIFIED` |
| PC 最小规则避障 | 固定 0.20 m/s；前缘 25 cm 停车后右优先 90°，左右同堵两段右转；侧边 15 cm 触发反向连续 20°弧线 | 当前规则取消覆盖、净空、扫掠与渐变速度判断；0 只表示未检出回波，不证明可通行；制动、转角和物理避障 `NOT VERIFIED` |
| RGB-D / MAP | HSV 或 YOLO 适配、ROI 深度、目标关联、MAP 管理、停靠及抓取侧选择；NX 另有最新接收数据模式 | 外参仍 `PROVISIONAL / NOT CALIBRATED`；时间配对、定位连续性、错误锁定和 USB/数据中断需按具体轮次诊断 |
| YOLO 三类别 | `color_block / rock_compact / rock_spiky`；已完成 V1/V2/V3 离线训练与开发集对比，V3 为 1063 train / 177 val | 真实 compact 训练框仍不足；同 57 张视频 B 为开发验证，独立最终 test 与 D435 跨场景泛化 `NOT VERIFIED` |

上表根据仓库源码、状态文档及既有实验报告整理；历史测试和部署记录不代表本次重新执行。构建、节点启动、DDS 发现、安全门 ARMED 或 API 返回不能单独证明真实运动有效。

最新证据入口：

- [NX 接近退出的回调修正](docs/go2_nx_near_odometry_callback.md)、[原接近与导航隔离](docs/go2_nx_original_near.md)、[NX 启动与最新数据模式](docs/go2_nx_full_startup.md)。
- [YOLO V2 报告](vision_training/reports/science_targets_mixed_v2/README.md)、[YOLO V3 报告](vision_training/reports/science_targets_mixed_v3/README.md)。V3 方块召回提高同时误检增加，不能直接推断所有场景都优于 V2。
- [感知与接近实验](camera_ws/docs/experiments.md)、[项目实验记录](docs/experiments.md)。

## 仓库入口

| 路径 | 用途 |
|---|---|
| [code/](code/) | GO2 DDS 接口、传感器变换、Point-LIO、地形、探索/规划、跟随与安全门 |
| [avoid_ws/](avoid_ws/README.md) | 独立最小规则雷达避障与离线回归 |
| [navigation_test/](navigation_test/README.md) | 独立自主导航往返、分区反应与 RViz2 |
| [camera_ws/](camera_ws/README.md) | 独立 D435 感知、map 目标管理与用户授权的搜索接近实验 |
| [vision_training/](vision_training/README.md) | 通用 YOLO 数据、训练、评估、导出与 PC ROS 2 推理 |
| [PROJECT_STATUS.md](PROJECT_STATUS.md) | 当前状态、历史基线与验证边界 |
| [TASKS.md](TASKS.md) | 阶段任务、阻塞和后续工作 |
| [docs/interface_inventory.md](docs/interface_inventory.md) | 导航主链 ROS 2 接口静态清单 |
| [camera_ws/docs/experiments.md](camera_ws/docs/experiments.md) | 感知与搜索接近实机记录、失败轮次与本地证据位置 |
| [vision_training/VALIDATION.md](vision_training/VALIDATION.md) | 训练环境、工程检查及已知环境问题 |
| [AGENTS.md](AGENTS.md) | 协作规则、证据等级和安全边界 |

## 环境与开始使用

PC 基础环境为 Ubuntu 22.04 / ROS 2 Humble。GO2 通过 ROS 2/CycloneDDS 发布 `/utlidar/cloud` 和 `/utlidar/imu`，本主线不需要额外启动 PC 侧雷达驱动。依赖与构建入口应按各工作流分别使用；全新主机完整复现仍待验证。

导航构建入口：

```bash
bash code/scripts/deploy_local.sh
```

只读传感器预检（先将连接 GO2 的有线网卡配置到 `192.168.123.0/24`）：

```bash
bash code/scripts/go2_lidar_preflight.sh <连接GO2的网卡名>
```

D435 构建和只读预览见 [camera_ws/README.md](camera_ws/README.md)；YOLO 的独立 venv、数据格式、人工审核和运行命令见 [vision_training/README.md](vision_training/README.md)。不要将训练依赖安装到系统 ROS Python 环境中。

真实运动需要逐轮明确授权、物理急停、空旷环境和可确认的停止路径；默认采用只读检查或 `--dry-run --no-arm`。独立搜索接近实验不等于 FAR/WP5 导航闭环已通过。构建成功、节点启动、DDS 发现、安全门 ARMED 或 API 返回均不能单独证明真实运动有效。

## 数据与复现

源码、配置、测试、报告与数据版本信息进入 Git；rosbag、原始图像/视频、大日志、训练权重、venv 和构建产物保留在本机并按 `.gitignore` 排除。单独 clone 仓库不会获得完整实验数据与权重。复现实验前需按对应报告准备原始数据、核对哈希和参数快照。

当前优先补充真实 compact、独立拍摄会话，以及弱光、边缘、小目标和干扰背景的人工标签，再评估泛化。未标注图不视作负样本；连续录像预测输出空档不能直接当作真实漏检，仍需人工逐帧标注。

## 历史导航基线

`code/` 主线来自 2026-08-31 回退版，后续工程改动以 Git 历史为准。原 2026-09-01 完整包中的 retest4/5/6/8/9 是历史实验，存在转向占比高、移动有限和走廊误杀记录，不能代表当前分支重新验收。数值与原始 bag 位置已保留在 [docs/experiments.md](docs/experiments.md)；旧引用 `today_representative.md` 未进入仓库。
