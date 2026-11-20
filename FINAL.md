> 历史集成说明（2026-11-17）：下文提交号和行为描述属于当时快照，未表示完整真机验收。最新 NX 修正与操作入口请以 [README](README.md)、[项目状态](PROJECT_STATUS.md) 和 [NX 最终命令](docs/go2_nx_final_commands.md) 为准。

# GO2 自主探索与多科学目标抓取 — 集成总说明

**分支：** `feature/science-targets-perception`
**最新Commit：** `bc9ff6d`
**日期：** 2026-11-17

---

## 一、项目背景与目标

本项目在已有单目标接近与抓取能力的基础上，完成以下最终集成：

1. 以自主导航（FAR + localPlanner）替代原有固定直线 SEARCH；
2. 在探索过程中持续检测科学目标，并通过 Mission Manager 自动触发导航→接近→抓取；
3. 建立统一运动控制仲裁机制，确保导航控制和接近控制不会同时争抢 `/cmd_vel_raw`；
4. 支持多目标连续处理循环，完成一个目标后恢复探索。

原有成熟模块（Point-LIO、FAR、localPlanner、ScienceTargetApproach、S1 抓取链）**均不重写**，本轮只新增胶水层。

---

## 二、整体架构

```
             GO2 UTLiDAR + IMU
                    ↓
               Point-LIO
                    ↓
           实时定位与建图 (map frame)
                    │
       ┌────────────┴───────────┐
       │                        │
  terrain_analysis          D435 + YOLO
  terrain_analysis_ext      科学目标检测
       │                        │
  FAR 全局规划             science_target_manager
  (接收 /goal_point)        (track 聚类 + MAP 坐标)
       │                        │
  localPlanner              /science/target_tracks
  pathFollower                   │
       │                         │
  /nav/cmd_vel_raw          Mission Manager
       │                   (状态机决策)
       │                         │
       └──────────┬──────────────┘
                  ↓
           Motion Arbiter
           (唯一 /cmd_vel_raw 发布者)
                  ↓
           go2_safety_gate
                  ↓
           /api/sport/request
                  ↓
                GO2
```

### 关键话题映射

| 话题 | 发布者 | 订阅者 | 说明 |
|------|--------|--------|------|
| `/goal_point` | MissionManager | FAR | 探索航点或目标停靠点 |
| `/nav/cmd_vel_raw` | pathFollower（remap） | MotionArbiter | 导航速度，仅 NAV 模式转发 |
| `/science/cmd_vel_raw` | ScienceTargetApproach（remap） | MotionArbiter | 接近速度，仅 APPROACH 模式转发 |
| `/cmd_vel_raw` | MotionArbiter | go2_safety_gate | **唯一**运动出口 |
| `/mission/motion_mode` | MissionManager | MotionArbiter | `NAV` / `APPROACH` / `IDLE` |
| `/science/target_tracks` | ScienceTargetManager | MissionManager | 已确认目标列表 |
| `/science/approach/start` | MissionManager | ScienceTargetApproach | 触发接近 |
| `/science/approach/confirm_target_id` | MissionManager | ScienceTargetApproach | 锁定目标 ID |
| `/science/approach/status` | ScienceTargetApproach | MissionManager | 接近结果反馈 |
| `/science/visited_target_id` | MissionManager | ScienceTargetManager | 标记已处理目标 |

---

## 三、新增模块说明

### 3.1 MotionArbiter（运动仲裁器）

**文件：** `camera_ws/src/go2_science_perception/go2_science_perception/motion_arbiter.py`

**核心逻辑：**
- 订阅 `/nav/cmd_vel_raw`（导航速度）和 `/science/cmd_vel_raw`（接近速度）；
- 订阅 `/mission/motion_mode`，按模式决定转发哪一路：
  - `NAV` → 转发导航速度；
  - `APPROACH` → 转发接近速度；
  - `IDLE` → 发布零速；
- 超过 0.4 秒未收到指令则自动发零速（防悬停）；
- 以 20 Hz 固定频率输出，确保 go2_safety_gate 始终有新指令。

**关键设计：** pathFollower 在源码中硬编码发布到 `/cmd_vel_raw`，通过 launch 文件 remap 改为 `/nav/cmd_vel_raw`；ScienceTargetApproach 同样 remap 到 `/science/cmd_vel_raw`。两者均**不知道**仲裁器的存在，仲裁器透明插入其间。

### 3.2 MissionManager（任务状态机）

**文件：** `camera_ws/src/go2_science_perception/go2_science_perception/mission_manager.py`

**状态机：**

```
IDLE
  ↓ /mission/start
EXPLORE  ──── 检测到目标 & 距离 ≤ 1.6m ──→ APPROACH
  │                                              ↑
  │  检测到目标 & 距离 > 1.6m                   │
  ↓                                              │
NAV_TO_TARGET                                    │
  ↓ 到达停靠点（距离 ≤ 1.2m）                   │
WAIT_NAV_ARRIVE（1s 稳定）──────────────→ APPROACH
                                               ↓ 接近成功(STOP/COMPLETED)
                                             GRASP
                                               ↓ 抓取结束
                                         RESUME_EXPLORE（2s）
                                               ↓
                                           EXPLORE / DONE
```

**探索策略：** `ExplorationWaypoints` 类生成以机器人当前位置为圆心、前方优先的扇形候选点，层层扩展（step=3m，共4层），避免重复访问已去过的区域。

**多目标记忆：** 已完成/已错过的目标 ID 存入 `_completed_ids` 集合，不会因视觉 track ID 刷新而重复处理。

**导航超时：** 60 秒未到达则标记 MISSED，恢复探索。

**抓取监控：** 320 秒未结束则强制退出（S1 task 自身超时为 300 秒）。

### 3.3 nav_stack.launch.py

**文件：** `camera_ws/src/go2_science_perception/launch/nav_stack.launch.py`

关键参数覆盖（原 launch 默认值均不合适）：

| 参数 | 原默认值 | 本文件覆盖值 | 原因 |
|------|---------|------------|------|
| `autonomyMode` | `false` | `true` | 默认关闭自主模式，不会自动行驶 |
| `maxSpeed` | `0.1 m/s` | `0.4 m/s` | 0.1 m/s 低于 GO2 有效响应门槛 |
| `autonomySpeed` | `1.0 m/s` | `0.4 m/s` | 与 maxSpeed 保持一致 |
| `goalCloseDis` | `0.4 m` | `0.6 m` | 更早判定到达，减少在目标点附近震荡 |
| `is_real_robot` | `false` | `true` | 实机模式 |

pathFollower 通过 Node-level `remappings` 将 `/cmd_vel_raw` 改为 `/nav/cmd_vel_raw`（IncludeLaunchDescription 不会传递 remap，必须直接声明 Node）。

### 3.4 autonomous_full.launch.py

**文件：** `camera_ws/src/go2_science_perception/launch/autonomous_full.launch.py`

一次性启动：Point-LIO + D435 + YOLO/HSV 检测 + ScienceTargetManager + ScienceTargetApproach（remap）+ MotionArbiter + MissionManager + go2_safety_gate。

ScienceTargetApproach 在此 launch 中加入 remap：
```python
remappings=[('/cmd_vel_raw', '/science/cmd_vel_raw')]
```

### 3.5 go2_autonomous_mission.py

**文件：** `scripts/go2_autonomous_mission.py`

PC 端编排器：
1. source ROS + deploy_ws + perception overlay；
2. 先启动 `nav_stack.launch.py`（等待 3 秒让 terrain 初始化）；
3. 再启动 `autonomous_full.launch.py`；
4. 监控子进程退出状态，Ctrl+C 触发逆序 SIGINT→SIGTERM 清理。

---

## 四、使用命令

### 离线计划/接口检查（无真实运动）

```bash
bash go2_test.sh --mode full --nav-mode autonomous --detector yolo --dry-run
```

输出启动计划、模块列表，不连接硬件，不写任何状态文件。

### 真实执行

```bash
# 确认：路线空旷、物理急停在手、S1 初始位已检查
bash go2_test.sh --execute --mode full --nav-mode autonomous --detector yolo
```

等价调用（参数等号或空格均可）：

```bash
bash go2_test.sh --execute --mode full --nav-mode=autonomous --detector=yolo
```

HSV 检测器（备用）：

```bash
bash go2_test.sh --execute --mode full --nav-mode autonomous --detector hsv
```

### 保留旧模式（单目标流程，不受本次修改影响）

```bash
# 单目标 dock 模式
bash go2_test.sh --execute --mode dock

# 单目标 full 模式（原有 SEARCH 5m + 接近 + 抓取）
bash go2_test.sh --execute --mode full

# 仅接近调试
bash go2_test.sh --mode approach
```

---

## 五、导航稳定性已修复的关键问题

| 问题 | 原因 | 修复 |
|------|------|------|
| GO2 不自主行驶 | `autonomyMode=false`（launch 默认） | nav_stack.launch.py 覆盖为 `true` |
| 速度指令无效 | `maxSpeed=0.1 m/s` 低于 GO2 死区 | 覆盖为 `0.4 m/s` |
| 导航和接近速度冲突 | 两者均发布到 `/cmd_vel_raw` | MotionArbiter 仲裁，remap 隔离 |
| 探索选点偏向身后 | 未约束前向区域 | ExplorationWaypoints 优先前半球候选 |

---

## 六、已知限制与 NOT VERIFIED 项

以下功能在代码层面已实现，但**尚未经过实机验证**：

- **自主探索闭环**：FAR + localPlanner 在真实未知环境中的避障表现；
- **NAV→APPROACH 切换时序**：实机切换时 MotionArbiter 的零速缓冲是否足够；
- **多目标连续抓取**：RESUME_EXPLORE 后第二目标的完整流程；
- **localPlanner 参数字段名**：`is_real_robot`、`goalCloseDis` 等字段需与当前 `deploy_ws` 编译版本的实际 C++ 参数名核对；
- **MissionManager 探索航点**：ExplorationWaypoints 的步长/层数是否与实际场地匹配；
- **go2_safety_gate 包名**：`autonomous_full.launch.py` 中引用了 `go2_safety_gate` 可执行文件，需确认在 `deploy_ws` 中的实际包名。

---

## 七、新增文件清单

```
camera_ws/src/go2_science_perception/
  go2_science_perception/
    motion_arbiter.py          # 运动仲裁器（新增）
    mission_manager.py         # 任务状态机（新增）
  launch/
    autonomous_full.launch.py  # 全模块 launch（新增）
    nav_stack.launch.py        # 导航栈 launch，含关键参数覆盖（新增）
  setup.py                     # 注册两个新入口点（修改）

scripts/
  go2_autonomous_mission.py    # PC 编排器（新增）

go2_test.sh                    # 新增 --nav-mode autonomous 入口（修改）
```

---

## 八、下一步建议

1. **实机冒烟测试**：在空旷区域先跑 `--dry-run`，确认所有节点启动、话题出现、TF 完整；
2. **localPlanner 参数确认**：`ros2 param list /localPlanner` 对照 nav_stack.launch.py 中的字段名；
3. **go2_safety_gate 包名确认**：`ros2 pkg list | grep gate`，修正 autonomous_full.launch.py；
4. **NAV→APPROACH 时序调优**：根据实机观察调整 `WAIT_NAV_ARRIVE` 等待时间（当前 1 秒）和 `NAV_ARRIVE_M`（当前 1.2 m）；
5. **BlockSelection 轮间重置**：在 MissionManager RESUME_EXPLORE 时发布 `/science/approach/reset`，防止旧目标锁残留阻断新一轮检测（已知历史 bug，见 `docs/go2_yolo_round2_selection_20261116.md`）。

---

## 九、同学复现环境搭建指南（同款硬件）

> **前提**：操作系统 Ubuntu 22.04，ROS2 Humble，GO2 + S1 机械臂 + D435/D455 相机，网络拓扑与本机一致（有线网卡直连 GO2，IP 段 192.168.123.x）。

### 快速初始化（推荐）

拿到打包目录后，先运行初始化脚本，它会交互式询问网卡名、GO2 IP、SSH 密钥路径、D435 序列号，并自动写入所有需要修改的文件：

```bash
cd ~/go2_deploy        # 或解压后的目录名
bash setup_for_new_machine.sh
```

脚本完成后，直接跳到**步骤 3**（重新编译 deploy_ws）。

---

### 手动步骤（脚本失败时参考）

### 步骤 1：安装基础依赖

```bash
# ROS2 Humble（已安装可跳过）
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions

# RealSense SDK 和 ROS2 wrapper
sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-description

# Python 依赖
pip3 install pyrealsense2 numpy opencv-python

# unitree SDK（需要与 GO2 固件版本匹配，向原始团队获取）
# 安装后确认 /opt/ros/humble/setup.bash 下能找到 unitree_go 包
```

### 步骤 2：克隆仓库

```bash
git clone --branch feature/science-targets-perception \
  https://github.com/Czx666666888888/GO2-Lidar-Itself.git \
  ~/today_full

cd ~/today_full
```

### 步骤 3：获取并构建 deploy_ws

`deploy_ws/` 包含导航栈预编译二进制（`point_lio_unilidar`、`far_planner`、`local_planner` 等）。**这些二进制与原机器编译环境绑定，不能直接复用**，需要从源码重新编译：

```bash
mkdir -p ~/today_full/deploy_ws/src
cd ~/today_full/deploy_ws/src

# 克隆导航栈源码（从 code/autonomy_stack_go2_src 目录获取）
cp -r ~/today_full/code/autonomy_stack_go2_src/* .

# 编译
cd ~/today_full/deploy_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -20
```

> **注意**：`point_lio_unilidar` 依赖 Eigen、PCL、livox_ros_driver2，编译前需要先安装：
> ```bash
> sudo apt install -y libeigen3-dev libpcl-dev ros-humble-pcl-ros
> # livox_ros_driver2 需要从 Livox 官方 GitHub 单独克隆编译
> ```

### 步骤 4：构建感知包

```bash
cd ~/today_full
source /opt/ros/humble/setup.bash
source deploy_ws/install/setup.bash

/usr/bin/python3 -X faulthandler /usr/bin/colcon \
  --log-base camera_ws/.onekey/log build \
  --base-paths camera_ws/src \
  --build-base camera_ws/.onekey/build \
  --install-base camera_ws/.onekey/install/go2_science_perception \
  --packages-select go2_science_perception
```

### 步骤 5：修改机器特定配置

以下配置写死在代码中，**必须按照你们的实际硬件修改**：

| 文件 | 变量/参数 | 原值 | 改为 |
|------|-----------|------|------|
| `scripts/go2_full_test.py` | `HOST` | `unitree@192.168.123.18` | 你们 GO2 的实际 IP |
| `scripts/go2_full_test.py` | `KEY` | `/home/czx666/.ssh/id_ed25519_go2` | 你们的 SSH 私钥路径 |
| `scripts/go2_full_test.py` | `213622070598` | D435 序列号 | 你们 D435 的序列号（`rs-enumerate-devices` 查看） |
| `scripts/go2_autonomous_mission.py` | （继承上面） | 同上 | 同上 |
| `camera_ws/src/go2_science_perception/launch/d435_slam_target_validation.launch.py` | `realsense_d435.yaml` | 相机配置 | 确认序列号一致 |
| `go2_test.sh` | `enp12s0` | 有线网卡名 | `ip link` 查看你们的有线网卡名 |

修改 D435 序列号示例（`scripts/go2_full_test.py` 第 44 行附近）：

```python
# 原值
p = subprocess.Popen(['bash', sys.argv[1]], start_new_session=True,
                     env=dict(os.environ, GO2_D435_SERIAL='213622070598'))
# 改为你们的序列号
                     env=dict(os.environ, GO2_D435_SERIAL='你的序列号'))
```

### 步骤 6：配置 SSH 免密登录 GO2

```bash
# 生成密钥（已有可跳过）
ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519_go2

# 复制公钥到 GO2
ssh-copy-id -i ~/.ssh/id_ed25519_go2.pub unitree@192.168.123.18

# 测试
ssh -i ~/.ssh/id_ed25519_go2 unitree@192.168.123.18 echo OK
```

### 步骤 7：在 GO2（NX）上部署 S1 服务

S1 机械臂控制服务运行在 GO2 机载 NX 上，需要提前部署：

```bash
# 从原机器复制（或从团队获取）
scp -i ~/.ssh/id_ed25519_go2 -r <原机器>:/home/unitree/s1-arm-control-v4 \
  unitree@192.168.123.18:/home/unitree/

scp -i ~/.ssh/id_ed25519_go2 -r <原机器>:/home/unitree/s1-arm-py310 \
  unitree@192.168.123.18:/home/unitree/
```

### 步骤 8：确认 localPlanner 参数字段名

编译完成后运行一次确认参数名与 launch 文件一致：

```bash
source ~/today_full/deploy_ws/install/setup.bash
# 在另一终端先启动节点
ros2 run local_planner localPlanner &
sleep 2
ros2 param list /localPlanner
```

对比 `camera_ws/src/go2_science_perception/launch/nav_stack.launch.py` 中的参数名，如有不一致按实际字段名修正。

### 步骤 9：验证接口（干运行）

```bash
cd ~/today_full
bash go2_test.sh --mode full --nav-mode autonomous --detector yolo --dry-run
```

输出应列出全部节点计划，无报错退出。

### 步骤 10：真实执行

```bash
# 确认：路线空旷、物理急停在手、S1 初始化位已检查
bash go2_test.sh --execute --mode full --nav-mode autonomous --detector yolo
```

---

## 十、常见问题

**Q：colcon build 报找不到 `livox_ros_driver2`**
A：从 [https://github.com/Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) 克隆，放入 `deploy_ws/src/`，重新 build。

**Q：`ros2 pkg list` 找不到 `go2_safety_gate`**
A：这个包在 `deploy_ws` 里，需要先 `source deploy_ws/install/setup.bash`，或检查原编译是否成功。如果 `deploy_ws/src` 里没有这个包的源码，需向原团队获取。

**Q：D435 探测不到**
A：先用 `rs-enumerate-devices` 确认设备识别，再把序列号填入代码。USB 3.0 接口，避免使用 Hub。

**Q：GO2 SDK DDS 通信失败**
A：检查有线网卡名（`ip link`）和 IP（`ip addr`），确保 PC 有线 IP 在 `192.168.123.x` 段，且 `ip route get 192.168.123.18` 显示 `dev <你的网卡名> src 192.168.123.99`。如果 IP 段不同，修改 `go2_test.sh` 中的路由检查和 DDS 配置。
