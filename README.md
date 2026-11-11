# GO2-Lidar-Itself

基于 ROS 2 的 Unitree GO2 自主导航与科学目标感知实验项目，包含机载 UTLiDAR/IMU、Point-LIO、FAR/局部规划、安全运动门、D435 RGB-D 定位，以及可复用的 YOLO 训练与独立推理工作流。

更新日期：2026-11-11。当前研发分支：`feature/reusable-yolo-training`；本次整理依据代码基线 `5fa5cd0` 和仓库实验报告，未重新进行训练、构建或真机实验。

## 当前进展

| 工作流 | 已有实现与证据 | 验证边界 |
|---|---|---|
| 雷达导航 | UTLiDAR/IMU → Point-LIO → 地形分析 → WP5/FAR → 局部规划/跟随 → safety gate；主链接口已静态盘点 | 干净环境全量构建、完整导航闭环和重复性能仍 `NOT VERIFIED` |
| D435 / HSV | 蓝色多目标、蓝墙过滤、对齐深度粗定位、按原始时间戳转换 map、目标生命周期与 visited 空间记忆 | 外参仍 `PROVISIONAL / NOT CALIBRATED`；MAP 漂移根因未确认 |
| 搜索与接近实验 | 独立 SEARCH → STOP_CONFIRM → MAP 锚点接近；5 m / 0.25 m/s 搜索、6 s 总确认预算；已审计一轮用户实测 9.5 cm 停稳，另有用户“两次都可以”反馈 | 单场景现场证据；5 m 路线终点、5 m ODOM 保护边界、重复精度和外部速度真值尚未验收 |
| YOLO 工程 | 配置化数据准备、人工审核、按拍摄组划分、训练/评估/PT 与 ONNX 导出；独立 ROS 2 RGB-D 检测节点 | 与 HSV/运动链分离；真实 D435 连续推理、跨场景泛化及 Orin/NX 部署 `NOT VERIFIED` |
| blue_cube pilot V2 | 20 张人工标注，14 train / 6 val，同场景按视频分组；实际 80 轮训练，已能拟合训练图 | 小样本、无 test；训练阶段有 NaN 记录，不能作为部署验收 |

YOLO V2 在 `conf=0.25`、类别正确且 `IoU≥0.5` 的一对一匹配下：best 为训练 `13/0/1`、验证 `4/0/2`；last 为训练 `14/0/0`、验证 `4/0/2`（均为 TP/FP/FN）。详情见 [V2 对比报告](vision_training/reports/blue_cube_pilot_v2.md)，失败的 V1 结果见 [V1 报告](vision_training/reports/blue_cube_pilot_v1.md)与[失败分析](vision_training/reports/blue_cube_pilot_v1_failure_analysis.md)。这些是既有实验记录，本次未重跑。

## 仓库入口

| 路径 | 用途 |
|---|---|
| [code/](code/) | GO2 DDS 接口、传感器变换、Point-LIO、地形、探索/规划、跟随与安全门 |
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

当前优先补充独立拍摄会话及弱光、边缘、小目标和干扰背景的人工标签，再评估泛化。未标注图不视作负样本，剩余标注包仍需人工审核。

## 历史导航基线

`code/` 主线来自 2026-08-31 回退版，后续工程改动以 Git 历史为准。原 2026-09-01 完整包中的 retest4/5/6/8/9 是历史实验，存在转向占比高、移动有限和走廊误杀记录，不能代表当前分支重新验收。数值与原始 bag 位置已保留在 [docs/experiments.md](docs/experiments.md)；旧引用 `today_representative.md` 未进入仓库。
