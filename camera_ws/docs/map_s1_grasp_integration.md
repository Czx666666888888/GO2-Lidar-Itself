# 单目标 MAP 停靠与 S1 抓取集成

基线：`feature/d435-perception-workspace` / `e15c66ce2beb998ef9c73464b0bff72cbd4e2266`。
独立分支：`feature/map-approach-s1-grasp-integration`；未合入视觉伺服分支。

## 坐标与停靠几何

vehicle：X 前、Y 左；S1：X 后、Y 右，二维相对旋转为 π。
现有 TF 为 sensor → vehicle = (-0.30, 0, 0)。按用户测量 S1 在雷达后方 0.36m，
二维暂用 vehicle → S1 XY=(-0.06,0)。Z 未测量，不发布虚构的三维 S1 TF。
`map -> vehicle` 已包含 sensor 偏移，控制中不再重复减去 0.30m。

源码证据边界：base_odom 将 SLAM 位姿视为雷达中心；Point-LIO 同时使用 IMU 外参，
输入点云还有既有变换。`sensor` 与实际测量的物理雷达原点是否精确重合尚未验证。
保留用户 XY 暂定值，不修改 Point-LIO 或相机外参。S1 抓取高度仍由原抓取模块估计。

目标在 S1 推荐位置为 (0,+0.31)m，在 vehicle 中为 (-0.06,-0.31)m。
`vehicle_goal = target_map - R(theta) @ [-0.06,-0.31]`，theta 保留搜索初始航向。
例如目标 MAP=(1,2)：theta=0 时停车点=(1.06,2.31)；theta=π/2 时=(0.69,2.06)。
两者预测 S1 目标位置均为 (0,0.31)。每帧记录目标、机器人、期望位姿和预测 S1 XY。

## 控制与交接

复用旧 SEARCH / STOP_CONFIRM、MAP 目标关联、定位有效性、安全门、速度/制动参数。
新功能默认关闭；只通过 `science_grasp_docking.yaml` 启用。
新模式控制参考是 `MAP_GRASP_DOCK`，不是相机视觉伺服。
以机身坐标系下的二维位置误差同时控制vx、vy、wz，不再使用TURN_TO_POSITION/TRANSLATE切换：

```text
dx = x_goal - x;  dy = y_goal - y
ex = cos(yaw)*dx + sin(yaw)*dy
ey = -sin(yaw)*dx + cos(yaw)*dy
etheta = wrap(yaw_goal - yaw)
vx_raw = kp_linear * ex
vy_raw = kp_linear * ey
wz = clamp(kp_angular * etheta, ±max_angular_speed)
```

Kx=Ky，沿用原kp_linear与kp_angular，没有新增横移增益。
先按合成平面速度限制到原max_linear_speed（本配置0.30m/s），
再沿用原max_deceleration、braking_latency和实际平面速度计算制动速度上界。
对MAP中的平面指令向量施加加速度/减速度范数限幅，再转换回机身vx/vy，
避免转动机身时把历史速度误认成新方向。接近位置区时平滑制动；航向可同时调整。
STOP/FAULT后三个分量都为0。

普通MAP模式vy=0、原0.10m停车条件和运行时限不变；
SEARCH仍为5m/0.25m/s/60s，STOP_CONFIRM总预算6s。

## 当前完成条件和时间保护（2026-11-12，仅离线验证）

完成条件为MAP位置误差≤0.04m、航向误差≤10°（0.174532925rad），
并满足原静止反馈条件持续0.5秒。原推荐S1矩形
X=[-0.05,+0.05]、Y=[0.29,0.33]m只记录参考，不再拒绝停车。
仍输出predicted_target_s1_xy及predicted_in_grasp_region。
这些条件只代表导航停靠；原S1/D455模块仍须确认目标可观测、可达，不能保证抓取成功。

四种时间/可信度条件分别处理：

| 条件 | 当前处理 |
|---|---|
| 抓取停靠固定总运行时间 | core与runner均取消，不再因25s或SEARCH+25s计时失败 |
| 视觉丢失后的ODOM接续 | 原搜索配置5m/25s不变；到限仍FAULT，并记录剩余预算 |
| 定位、相机TF、Sport反馈新鲜度 | 原0.4s/0.5s等有效性检查保留；无效反馈、接口竞争和人工停止仍停止 |
| 无有效进展 | 新增dock_progress_timeout=10s；归一化位姿误差未累计改善0.25则FAULT并零速 |

进展量为hypot(max(0,d-0.04)/0.04, max(0,abs(etheta)-10°)/10°)。
改善门槛单轴时对应约1cm或2.5°，微小往复抖动不能续期；10秒是卡滞窗口，不是总任务时限。
runner另外监控控制状态心跳（1s）与子进程退出。
ODOM运动时间下界改为平移和转动所需时间的max，适配同时运动；预算本身不变。
因此长时间完全看不到目标时，即便停靠正在改善，仍可能在ODOM25s触发保护；
是否需要单独调整定位可信度策略，必须根据下一次实测的剩余路径、视觉丢失时刻及定位误差评估。
本轮未扩大ODOM预算，也未重复采集姿态高度。

vy正向按现有vehicle/键盘遥控约定为GO2左侧，S1的+Y则为GO2右侧。
安全门已有linear.y读入并发布Move JSON的y分量，不新增Sport控制节点，也不改安全门。
现有Sport反馈的XY用于hypot计算速度大小，方向闭环来自MAP->vehicle姿态；
本轮只静态核对接口与符号，真实横移方向和反馈跟随仍NOT VERIFIED。
参考：[官方SDK2 Move接口](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/go2/sport/sport_client.py)。

三自由度横移控制已实现，但尚未实机验证。历史两次vx/wz失败记录保留在experiments.md，
不能作为新控制器的运动证据。

完整模式先退出本轮控制器和安全门，确认没有其他速度发布者，再交给 SDK2 SportClient。
StandDown/StandUp 通过已有 SDK2 接口；S1 只通过原 `arm_task_client.py` 和
`/tmp/s1-arm-integration.sock` 调用。没有另开串口或重写抓取。
accepted=true 仅代表接受；须等待新 last_log、last_exit_code=0、正确 last_result、
busy=false、ZERO_HOLD、navigation_allowed=true，才能起立。
FAULT_HOLD/FAULT_IO、超时、PC 心跳丢失或 /stop 均不自动起立。
抓取结果中的 placement_confirmed 保持 null，不宣称已入盒。

## 交付包与实际连接

交付包：S1_arm_integration_20261111_auto_zero_hold_v1.zip。
SHA256：`a7e08d875366dc777e2a41e19401f537fa929584c9c89ae6a0ceaea5aab6596e`。
已原样解压到 GO2 `/home/unitree/s1-arm-control`；未修改绑定 JSON、标定或 SDK。
2026-11-12 SDK 实际枚举：D435 `213622070598`，D455 `231622302209`。
D455 与交付包标定序列号一致；先前不同序列号是换接前的枚举结果，不能用于当前设备。
串口 `/dev/ttyUSB0`（CH340）；用户已完成对时。
旧自定义 `go2-remote-router.service` 已停止，原厂遥控不在此次停用范围。
S1 第一次启动因 J5=-0.126841rad 超出原±0.08rad窗口被拒；用户恢复零位后第二次进入 ZERO_HOLD。

## 命令

本轮隔离工作树为 `/tmp/go2_map_s1_integration_20261111`。
先构建本包（本轮已完成），不做全仓库构建：

```bash
cd /tmp/go2_map_s1_integration_20261111
source /opt/ros/humble/setup.bash
source deploy_ws/install/setup.bash
colcon --log-base /tmp/science_s1_log build --base-paths camera_ws/src \
  --build-base /tmp/science_s1_build --install-base /tmp/science_s1_install \
  --packages-select go2_science_perception
```

GO2 机械臂服务（已启动时不要重复执行；首次使能会回零）：

```bash
ssh -i /home/czx666/.ssh/id_ed25519_go2 unitree@192.168.123.18
cd /home/unitree/s1-arm-control
OPENBLAS_NUM_THREADS=1 /home/unitree/s1-arm-py310/bin/python -B scripts/run_arm_integration.py
OPENBLAS_NUM_THREADS=1 /home/unitree/s1-arm-py310/bin/python -uB scripts/run_arm_integration.py --dev /dev/ttyUSB0 --execute
```

另一 GO2 终端仅查状态，不单独发送 grasp：

```bash
/usr/bin/python3 /home/unitree/s1-arm-control/scripts/arm_task_client.py status
```

相机和 PC 感知定位链（当前已启动，不要重复执行）：

```bash
# GO2；明确绑定 D435，D455 留给 S1
GO2_D435_SERIAL=213622070598 bash /home/unitree/go2_map_s1/launch_go2_d435_foxy.sh
# PC，新终端
cd /tmp/go2_map_s1_integration_20261111
source /opt/ros/humble/setup.bash
source deploy_ws/install/setup.bash
source /tmp/science_s1_install/setup.bash
source camera_ws/scripts/science_approach_pc_env.bash
ros2 launch go2_science_perception d435_slam_target_validation.launch.py remote_camera:=true use_rviz:=false
```

仅打印完整计划，不开硬件：

```bash
bash camera_ws/scripts/run_map_s1_task.sh --full
```

真实单独停靠（会移动 GO2，须空旷区域、单个目标、遥控急停在手）：

```bash
bash camera_ws/scripts/run_map_s1_task.sh --execute --search-distance 5 --search-timeout 60
```

完整流程（会移动GO2、趴下、抓取、起立；本轮已经完成姿态参数实测）：

```bash
bash camera_ws/scripts/run_map_s1_task.sh --full --execute \
  --search-distance 5 --search-timeout 60 \
  --posture-profile camera_ws/src/go2_science_perception/config/science_s1_posture_go2_20261112.json
```

实测配置为趴下反馈高度上限0.084m、起立下限0.304m，两种反馈mode都为0。
阈值来自本次稳定采样范围外扩1cm；它们不是S1基座物理高度，也不独立证明腹部支撑。
本轮趴下稳定已由用户现场确认；更改安装或支撑构型后需要重新核对。
`--docking-geometry-verified`只记录实际XY验证，未做测量不传该参数。

GO2 SDK2 Python编译绑定的DDS与系统默认DDS不同：系统默认库写请求会段错误。
协调器仅给姿态子进程指定以下环境，不更改系统库、相机或S1服务环境：

```bash
CYCLONEDDS_HOME=/home/unitree/cyclonedds_ws/install/cyclonedds
LD_LIBRARY_PATH=/home/unitree/cyclonedds_ws/install/cyclonedds/lib
```

可用`--sdk-cyclonedds-home`指定另一台机器实测有效的匹配安装目录。
趴下前仍必须StopMove成功；已经趴下、静止且S1恢复ZERO_HOLD后直接StandUp，
因为本机趴下时StopMove返回-1，不把行走停止API作为起立的必要条件。

PC 软件停止（物理遥控急停仍须保留）：

```bash
ros2 topic pub --once /stop std_msgs/msg/Int8 '{data: 1}'
```

不要在 GO2 行走或抓取中停止 S1 常驻服务；退出后软件无法保持机械臂。
完整任务失败不自动再次尝试。实验结果和未验证项见 experiments.md 本轮记录。
