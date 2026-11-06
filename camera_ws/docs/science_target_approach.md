# 单目标接近实验（不依赖 FAR）

本轮用户授权替代既有“仅可视化”边界，新增独立接近控制器。后续用户逐次明确授权
短时及完整实机测试，结果见 experiments.md；这不授权未来自动重启运动。
默认 launch 强制 `dry_run=true, allow_odom=false`，仅发布
`/science/approach/cmd_vel_preview`，不会发布 `/cmd_vel_raw`、武装或 Sport 请求。
本流程不启动 FAR、WP5、localPlanner、pathFollower，也不调用完整导航脚本。

## 数据与坐标

复用既有 D435 蓝色检测、蓝墙过滤、时间戳 TF 转换和 manager 的 visited 空间记忆。
manager 新增只读 `/science/target_tracks` (`std_msgs/String`, JSON)：
`frame` 和每个 track 的 `id/state/position/observed_ns/first_seen_ns/visited`。
`observed_ns` 是原始相机观测时间，不能用 selected/standoff 的重发时间代替。
既有 selected/standoff/Marker 接口和识别算法不变。

控制器按 `/base_state_estimation` 的原始时间查询完整 pose TF 到 map，包括位置和朝向。
它是 `base_odom_node` 已减去雷达前向 0.30 m 偏置的**机身中心**，不是雷达中心。
该节点继承 `camera_init` header 和 `aft_mapped` child 名，不能仅根据 frame 名判断中心。
Point-LIO 当前源码不填写 twist，因此实测速度和朝目标方向速度来自带时间戳的
机身中心 map 位姿，默认在 0.5 s 窗口内差分；不是命令速度。
ETA 仅在实测朝目标速度大于 0.01 m/s 时有数值，否则为 null。静止定位漂移可能产生
非零速度/有限 ETA，不能将该数字视为外部真值或物理运动证明。

默认停靠距离 0.80 m、容差 0.03 m，距目标的是机身中心到粗定位表面点的 map XY 距离。
它是首轮保守参数，**不代表已测量机身净空或完成外参标定**。真实运动前仍需测量
机身前缘、相机安装及目标尺寸；现有外参为 PROVISIONAL / NOT CALIBRATED。
manager 的旧 0.40 m standoff 仅用于可视化，控制器不使用它作为停靠距离。

## 状态和互锁

人工确认时选择最近的 ACTIVE CONFIRMED、未访问、新鲜目标，轨迹至少存活 3 s；
该时长门槛不能消除持续误检。IDLE status 输出 candidate 供现场核对。锁定后只更新
同一 ID，移除/合并、visited 改变或目标跳变进入 FAULT，不跟随新 selected。
LOCK_TARGET 后依据 yaw 进入 ALIGN 或 APPROACH_VISUAL；大角度误差时前进为零。
距离比例限速加加速度上限，制动上界解 `v²/(2a)+latency*v <= clearance`；
参考实际速度估算制动距离。终点进入 STOP，异常进入 FAULT，两者均锁存。

视觉超时默认 0.5 s；第一阶段和首次真实运动都禁止视觉丢失后继续走。
APPROACH_ODOM 仅可离线/dry-run 显式开启：用最后有效 map 目标加当前实测位姿闭环，
默认最多继续 1 s 或累计位姿路径 0.15 m，任一到限即 FAULT。真实模式初始化拒绝
`allow_odom=true`，阶段二验收后才另行解除这一代码门槛。

检查定位数据和控制循环超时、非有限值、姿态四元数、原始时间倒退/重复、TF 缺失、
map 位姿/朝向跳变、ROS 时钟回退、GO2 状态超时、目标距离与实验总时限。
future_tolerance 默认0.10 s：本轮90样本测得位姿stamp超前46.8–54.7 ms，旧0.05 s门槛
曾在运动前拦截；当前显式留100 ms预算，仍拒绝更大超前、重复或倒退，不修改原始stamp。
无法从无跳变的连续数值识别所有 map 重置，重置定位/地图必须人工停止并重新初始化
整条实验链；当前没有上游 map epoch 消息可消费。
启动期 TF 尚未发现也可能锁存 FAULT，必须等 TF 就绪后人工 reset。

真实模式只发布 `/cmd_vel_raw` TwistStamped，持续检查发布者数量。
确认前及运行中查询 `/go2_safety_gate` 参数，要求最低速度为 0、速度不高于本控制器上限、
lease 开启、gate 为真实模式、gate 加减速度能力不低于控制器假定减速度、命令超时不大于
制动延迟预算，且本机可发现的 Sport ROS 发布者仅为 gate；验证过期即停车。
这不能证明机器人内部 bare-DDS 输出者的行为，控制权互锁继续由 gate 处理。
专用 `science_approach_safety_gate.yaml` 保留 dry-run 默认，max_speed=0.30、min_speed=0。
本轮没有修改 gate 源码或禁用已有 lease/急停/超时/加速度互锁。
STOP/FAULT 发零 Twist 和 `/stop` Int8=1，只有 gate 将其转成 StopMove；控制器不直接发 Sport。
正常退出尽力发送停止；崩溃/SIGKILL 依赖仍在运行的 gate 命令看门狗（配置 0.30 s）。
单元测试不是实机故障停车证据；本轮视觉丢失实际停车另有用户确认，其他故障物理
停车仍需分别验收。

## 阶段一运行方式

先构建 camera_ws 并 source 实际 overlay；下列部署工作空间和网卡名为本机路径。

```bash
source /opt/ros/humble/setup.bash
source /home/czx666/today_full_20260901/today_full/deploy_ws/install/setup.bash
source /home/czx666/today_full_20260901/today_full/camera_ws/install/setup.bash
cd /home/czx666/today_full_20260901/today_full
source camera_ws/scripts/science_approach_pc_env.bash
ros2 launch go2_science_perception d435_slam_target_validation.launch.py remote_camera:=true use_rviz:=false
```

GO2 的相机只需单个驱动进程。官方 wrapper 4.54.1（源码 commit
`ae45d4969f5a7105a6a9e0deeadc785004d72441`）在
`/home/unitree/go2_d435_foxy_ws` 源码构建，链接系统 librealsense 2.54.2。
Foxy profile 名和 PC wrapper 不同，配置使用 `realsense_go2_foxy.yaml`。
将该 YAML 及 `camera_ws/scripts/launch_go2_d435_foxy.sh` 复制到 GO2 工作空间，SSH 登录后：

```bash
bash /home/unitree/go2_d435_foxy_ws/launch_camera.sh
```

脚本限制远端仅与 PC `192.168.123.99` 发现通信，PC 配置对应 peer 为 GO2
`192.168.123.18`，以避免 Foxy 在原生 GO2 DDS 发现下的启动异常。
只 remap 话题，不修改图像内容、原始 stamp 或 optical frame。
启动前两机需实际对时；NTP active 不等于 synchronized。
本轮通过用户 sudo 执行局域网一次性对时，未安装系统授时服务；重启/长期漂移后必须重查。

另一个 source 了同样环境的 PC 终端：

```bash
ros2 launch go2_science_perception science_target_approach.launch.py
ros2 topic echo /science/approach/status
```

确认 candidate ID、位置与实物对应、目标及机身保持静止，且 `/cmd_vel_raw` 没有发布者。
例如核对 candidate=1 后显式确认：

```bash
ros2 topic pub --once /science/approach/confirm_target_id std_msgs/msg/Int32 '{data: 1}'
```

若 candidate 已变化则拒绝确认。空消息 `/science/approach/confirm` 仅兼容 dry-run，真实模式拒绝。
终止预览可发 `/stop` Int8=1（会影响同域已有 gate，确保当前无真实输出）；
复位采用 `/science/approach/reset` Empty，清空旧观测与锁定后等待新鲜数据，再人工确认。
重置不会自动武装 gate。默认 JSONL 在 `/tmp/science_approach_<timestamp>.jsonl`，
逐控制周期记录参数、初始/当前距离、位置、距离误差、yaw 误差、命令/实测速度、ETA、
运行时间、视觉丢失、状态和故障原因；参数 `log_directory` 可改为持久目录。

## 验证及真实运动门槛

离线功能测试：source ROS 后将 package 加入现有 PYTHONPATH，运行相关 pytest。
实际 DDS 合成接口测试需 source deploy_ws 并显式使用隔离域：

```bash
ROS_DOMAIN_ID=86 RMW_IMPLEMENTATION=rmw_fastrtps_cpp SCIENCE_ROS_TEST=1 \
  PYTHONPATH="$PWD/camera_ws/src/go2_science_perception:$PYTHONPATH" \
  /usr/bin/python3 -m pytest camera_ws/src/go2_science_perception/test/test_approach_ros.py -q
```

不得在机器人域运行合成数据注入。合成停车不代表实机停车。
2026-11-06 调参110/250后，目标选择与静态输入超时检查通过。用户随后要求直接进入
受限实机，未完成原计划的人工动态dry-run；完整轮因视觉丢失保护停车，尚未完成
0.80 m可见目标停靠验收，详见 experiments.md。

后续真实测试需重新明确授权。用户需确认场地平整无障碍、物块身份与净空、
GO2 模式/控制权、物理急停和监护。先短距离、较低速度，逐渐提高到 0.30 m/s；
现场检查 gate 专用参数，禁止其他持续 Sport 控制器和 raw 发布者，分别人工武装与确认具体 ID。
本轮用户已确认视觉丢失停车无明显异常；到点精度、外部真值速度、进程死亡物理停车
仍需另采证据。现有里程计差分曾峰值0.494 m/s，不能把0.30命令上限当成实测速度上限。
本文件没有自动启动真实 gate 或自动武装的脚本。
