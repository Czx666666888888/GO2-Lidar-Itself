# Project Status

## 2026-11-14 S1兼容ttyUSB0/ttyUSB1

来源：用户指令、启动脚本与18项离线接口测试。入口仅枚举0/1，唯一存在时传给原S1服务；
缺失/双端口阻断。已有服务从/proc命令行核对绑定，不一致/多所有者保留原服务并阻断，
不自动停保持或创建第二串口所有者。串口名不证明物理设备身份，仍需原SDK/反馈预检。
本轮仅脚本/文档及离线测试，远端服务未重启、未发动作，实机通信恢复NOT VERIFIED。

## 2026-11-14 停靠请求速度下限0.20m/s

用户要求将停靠速度下限提高到0.20m/s；仅修改集成停靠配置`dock_min_linear_speed`，
从0.10改为0.20。它约束容差外的平面请求速度模长，制动/实测速度/向量加减速限制优先，
因此最终命令仍可低于0.20。0.30m/s上限、0.25m/s搜索、几何和停稳/趴下条件不变。
37项停靠及接口离线检查、单包colcon构建通过，已安装YAML读回0.20且与源码逐字节一致。
本轮未执行机器人动作；实际响应、停车精度及抓取NOT VERIFIED。

## 2026-11-13 快速启动与S1实测零位接口（基线f0bfd6ce152a）

本轮用户确认GO2暂时没电，只完成代码；未部署/启动设备、未回零、未GO2运动。

- 默认PC运动入口跳过原20s在线+20s暂停的固定采样；仍临时暂停/恢复原timesyncd、
  检查服务意外重启并用实时IMU映射READY/时间戳验收。完整采样保留显式--clock-diagnostic。
- 取消默认8s原生独立采集+原ros_probe+30s完整定位诊断串行组合。只读启动探针按连续
  数据条件提前结束：原生频率/丢包/扫描跨度、共享映射READY、150ms未来/400ms新鲜度、
  时间单调、有限位姿、无/clock、Point-LIO use_sim_time=false、精确机身/相机TF。
  dock还要求实时CONFIRMED未访问目标与selected一致；未修改识别/MAP/停靠算法或参数。
- 现有健康D435/Point-LIO/base/TF和新版S1 ZERO_HOLD直接复用；新建节点才归本轮退出清理。
  健康且仍新鲜的结果不重复验雷达/TF；冷启动最终只补定位/相机链检查。
  失效时间映射、过期定位重建已识别的原launch会话；未知外部进程所有权阻断而不猜杀进程。
  失效相机只停止已确认serial的旧驱动后重新准备时钟；不在健康相机运行时改它的时间。
- 构建缓存按包源码/配置内容hash，不再按整个Git commit；同一marker在启动器/诊断脚本共享，
  原同commit缓存可迁移，文档和启动脚本修改不触发无关包重编译。此次没有修改ROS包算法，未重编译包。
- 合并远端设备/进程检查、SSH短期连接复用；S1等待只用一个客户端通过原Unix socket本地轮询，
  原生观察与S1准备并行。阶段耗时写full_*/startup_timing.json，输出STAGE和SLOWEST_STAGE，
  PC时间上下文另记time_context_ready_sec。45秒是启动最大等待，READY/具体故障立即返回。

原S1源码来自交付ZIP，SHA256=a7e08d875366dc777e2a41e19401f537fa929584c9c89ae6a0ceaea5aab6596e，
与既有仓库记录一致。补丁位于camera_ws/s1_service，只更新原run_arm_integration、integration、
stdlib客户端及状态helper。安装只在服务未运行时进行，校验原件hash、备份并拒绝覆盖未知修改。
不创建第二个串口控制进程；D455抓取/入盒流程、SDK、原Arm.move实现、原绑定路径不改。

S1状态保留原state，增加readiness_state/position_state：COMMUNICATION_ERROR（带原I/O/SDK错误）、
NOT_AT_ZERO、AT_ZERO、ZERO_HOLD、BUSY、FAULT。使用唯一服务的get_pos/get_vel/get_temp读取六关节，
零目标沿用ZERO_Q，零位容差沿用原max_track_err_rad并受原start_safe_windows_rad约束；
反馈读取时间不等于独立电机包新鲜度，SDK未暴露后者，不能据此宣称线缆故障必能立即被检测。

通信正常且在原安全起步窗口内的非零位：同一服务排队zero，沿用原startup_zero S曲线插值，
返回后连续有效反馈检查并再次确认六关节零位才进入ZERO_HOLD；已有保持不再次回零。
GRASPING/RETURNING_ZERO拒绝重复zero。超出原窗口（如历史J5=-0.126841rad）仍单独报告NOT_AT_ZERO，
return_zero_allowed=false，不扩大范围、不猜测安装空间可用路径。真实串口错误不发送回零。
已有旧服务若仅返回摘要、没有实测关节接口，保留服务并阻断升级；不为读取位置重新连接串口。
下一次服务停用后的原一键入口会校验安装接口；未知远端版本需核实，不能强制覆盖。

验证：10项针对性接口检查通过（不导入SDK、不打开串口、不发动作），py_compile/bash语法/diff通过。
检查覆盖默认不调用40s采样、显式诊断保留、非零位与I/O分类、BUSY去重、ZERO_HOLD复用及
只读启动不创建/使能设备。无全仓构建、无离线仿真/批量测试。真实串口状态、六关节值、
回零路径实机、冷/热启动总秒数均NOT VERIFIED：GO2没电；不能把软件耗时当作实机提速结果。
原最大固定开销由源码确认是40s采样，另有8s原生和30s诊断；故障时S1另等最多45s。

唯一正常运动入口仍是：bash go2_test.sh --execute --mode dock。
只读计时/已有服务状态选项：bash go2_test.sh --startup-check --mode dock（不回零/不启动相机或定位）。
显式完整时钟诊断：在原运动命令后加--clock-diagnostic；已有--time-diagnostic --pc-time-check
只读完整诊断入口也保留。本轮未执行任何--execute命令。


## 2026-11-13 右侧物块横向净距调整（静态）

用户确认前后到位但右侧过近，要求机身再远离20～35cm；本轮取中值27.5cm。
集成停靠Y目标由S1右侧0.31m改为0.585m，机身停车位向左移0.275m；前后位置、
4cm/10°/0.5s、速度和S1动作链不变。几何校验允许停车目标在抓取参考区外，
原29～33cm抓取参考区保持，predicted_in_grasp_region=false不伪称可抓取。
默认DockSettings历史值保留，真实入口通过science_grasp_docking.yaml覆盖；计划输出同步。
新版仅定向软件检查，真实净距/机械臂可达性NOT VERIFIED；下一轮先用dock，不直接full。
命令：bash go2_test.sh --execute --mode dock（真实运动，本轮未执行）。


## 2026-11-13 单次dock失败收尾与full入口静态接入

用户现场确认已停稳但位置有偏差。实际单次dock在19.600s因无进展保护FAULT，末端129mm；
曾短暂35.36mm但未完成停稳，不构成成功。未趴下/抓取，PC校时恢复、S1保持；视频保留本地。
按用户要求移除抓取停靠的progress_timeout退出，继续闭环直至原位姿/停稳条件或独立保护触发。
原25s/5m丢视觉ODOM、新鲜度/异常停车/人工急停均保留；未改增益/速度/容差/S1动作链。
同一PC时间上下文现支持full，自主SEARCH→停靠→StandDown→抓取→ZERO_HOLD→StandUp；
full无需启动时目标可见，dock仍需目标预检。36项定向检查、单包构建通过；新行为未真机复测。
受控期间跳过可能激活timesyncd的D-Bus查询，并监测意外重启；此前全程暂停说法不作因果证据。
源码/实测边界见experiments最新节及go2_dock_stall_20261113摘要。

## 2026-11-13 PC受控IMU定位验收通过（NX暂停）

PC在线/暂停校时20秒对照未复现历史395ms差，NTP唯一因果未确认；不修改内核频率。
修复相机伴随主机设时子进程延迟路径后立即检查约+2.9ms；不改GO2原装主控时间。
临时暂停timesyncd下IMU会话映射60秒PASS：原生14.816Hz、Point-LIO/base889条，
机身时间滞后16.5～59.9ms，机身/相机TF100%，目标MAP470/470，最终新鲜CONFIRMED ID1。
不需在线offset修正；PC服务已恢复active，S1保持空闲。未运动或抓取。
唯一dock入口增加同一受控暂停/恢复上下文并默认IMU，原算法/阈值不变；新增运动分支未实机执行。
证据go2_pc_controlled_20261113，命令手册及experiments最新节。

## 2026-11-13 NX本机时间适配

NX实查Ubuntu20.04/Foxy/aarch64，缺Point-LIO/transform/base/perception四包；已有D435与CycloneDDS。
本机原生DDS实测14.883Hz/249.819Hz；完整定位/TF/MAP没有启动，NOT VERIFIED。
新增local主机入口与env配置，时间取节点所在主机，每会话重新映射、共享LiDAR/IMU时间、不依赖PC。
记录漂移；时钟突变停止输出，原新鲜度保护请求停机，重新启动链建立新会话；未实测物理停止。
NX独立入口已部署并验证缺包阻断。完整NX运动/S1编排仍待迁移，不把PC专用执行器冒充NX可用。
推荐保留宿主、准备ARM64 Jammy/Humble本机容器；未升级或安装。详见手册与experiments最新节。

## 2026-11-13 18:13 legacy真实一键验证（源码b3b603a）

原生健康后真正运行legacy60秒，Point-LIO/base883条且流连续；机身TF883/883、相机TF887/887。
目标MAP316/316，管理曾CONFIRMED，但最终目标STALE；机身超前363～766ms，大于150ms。
转换后IMU即有约765ms超前，非Point-LIO或base追加；固定首帧offset大于后续原始IMU接收差。
采集中PC ROS增量60.000s、MONOTONIC_RAW59.605s、GO2源59.619s，PC计时相对RAW显著变化；
采后NTP延迟736ms/抖动857ms，仅支持检查PC计时环境，尚未完全区分初始化延迟与校时变化。
不放行运动，未改算法/容差/系统时钟服务；证据go2_legacy_validation_20261113与experiments最新节。

## 2026-11-13 恢复a8f7c667历史legacy模式（实施基线45eac367）

仅恢复首帧LiDAR接收-header固定offset、LiDAR/IMU共用、点内时间不变，legacy绕过新版IMU锁存。
默认legacy，IMU可选代码保留；Point-LIO算法、D435/MAP管理、SEARCH/STOP_CONFIRM、3DOF/S1均保留。
用户明确要求future_tolerance由0.10改0.15，已同步默认/YAML/安装包；400ms新鲜度不变。
两受影响包各构建部署一次，安装源码一致。原生实测cloud8.150Hz/IMU249.988Hz，源端8.175Hz/
35.359%丢包，检查未通过；按用户要求未启动legacy定位/D435/MAP，运动恢复NOT VERIFIED。
只读一键：bash go2_test.sh --time-diagnostic --legacy-check；原dock运动入口保留，当前不放行。
证据camera_ws/analysis/go2_legacy_restore_20261113；当前legacy可能仍带历史约57ms超前，不是精确同步。

## 2026-11-13 蓝色目标运动前验收（cbf71eb基线）

20ms调整为诊断报告目标，硬检查对齐现有100ms future/400ms odom freshness，未改任何算法/阈值。
用户实际放置蓝色目标；伴随主机时钟重置到2022，现有脚本用户输入密码后立即对时通过。
目标MAP精确440/440、selected339条，管理曾CONFIRMED；但原生源再次2.066Hz/83.425%丢包，
Point-LIO/base仅70条、最大间隔737ms，映射最终IMU时间保护锁存、READY=false，目标STALE。
定位与目标链不持续，不能恢复运动；没有启动运动/抓取。--mode dock运动前增加真实目标验收。
源码/实测/失败边界见camera_ws/analysis/go2_blue_target_20261113与experiments最新节。

## 2026-11-13 上游雷达复查（cc375e6基线）

cloud_scan_num正常/异常/恢复全为180；没有依据改成18。异常源端自报1.213Hz/86.188%丢包，
PC1.225Hz及单消息扫描717ms，支持原生生产端退化而非PC单独漏收，触发根因未确认。
未发送配置/重启，用户确认未操作；当前自行恢复，最终PC14.771Hz、源端14.838Hz、丢包1.105%。
最终30秒443条Point-LIO/base，精确机身TF443/443、相机TF447/447；base超前中位16.394ms，
连续里程计在该窗口恢复。扫描结束中位20.260ms略超原20ms诊断线、无有效蓝色目标MAP样本，
仍不放行运动；此前98条后触发IMU保护的失败保留。时间映射/阈值/MAP/S1算法未改。
原生只读健康检查整合进一键入口与运动前检查；官方直连SDK接口不等于GO2 DDS配置方法，未猜命令。
证据及边界见camera_ws/analysis/go2_native_lidar_20261113和experiments最新节。

## 2026-11-13 实机复验未通过（b5ca598基线）

真实部署并启动只读链。雷达原生点云中位1.213Hz/86.188%丢包，长扫描触发转换器年龄保护；
最终offset +3454302.679386230s，正式窗口state/base均0条，里程计误差不可测。
修复诊断图像话题及DDS peer后收到458帧D435，精确相机map TF 0/450，目标MAP未通过。
伴随主机用户对时后仍观测不稳定时差（+136.563/-109.036ms）；源时钟未修复。
未运动，未放宽阈值、未改时间映射/MAP/三自由度/S1算法。失败后已清理本轮进程。
先恢复原生雷达输入和稳定主机对时，复验通过后才可`bash go2_test.sh --execute --mode dock`。
证据：camera_ws/analysis/go2_hardware_20261113；手册与experiments末节。NOT VERIFIED。

## 2026-11-13：IMU辅助时间映射代码交付

基于fetch确认的bac3a992feb7，实现2秒/200样本IMU窗口P5偏移，初始化后冻结b，
cloud/IMU统一t_local=t_sensor+b；保留点内时间和0.10s保护，检查扫描结束时间。
一键入口接入只读链、精确TF、offset残差/里程计诊断；原生观察用--observe-only。
原始发布端时钟没有改变，未知单向延迟仍无法与真实钟差分离；这是近似映射。
本轮按用户“先完成代码部分”仅做5项边界检查及Python/Shell/XML静态检查，未构建/部署/实机运行。
实际offset、旧57ms超前是否消除、持续定位、D435目标MAP转换和Orin兼容均NOT VERIFIED。
Point-LIO算法/滤波器及MAP/S1控制链未改。当前命令与下一轮验证见GO2实机测试命令手册.md最新节。

## 2026-11-12后续：纯PC DDS/LidarState诊断，不以源主控SSH为前提

实际fetch最新Push仍为12d0db5；本轮未更改时间转换算法/保护、稳定MAP或视觉伺服分支。
Domain0/enp12s0发现并读取/utlidar/lidar_state（unitree_go/msg/LidarState），30秒149条；
同时446帧raw cloud、7491条raw IMU。用户确认L1，实际版本firmware1.0.2/software1.0.0.38/SDK1.0.9。
三条原生Topic的DDS participant prefix相同；DDS源时间、状态stamp、serial_recv_stamp均旧约3390496.05秒。
确认偏差存在于发布端时间域，A（发布者侧时钟）最吻合，但公开状态不足以完全区分OS时钟与驱动显式改写。
L1接口仅查到TIMESYNC类别/getTimeDelay读取，未找到当前DDS可用的公开主控设置时间API；
RobotState公开接口没有该方法，Config通用文件接口也没有公开的校时配置键。未套用L2同步API。
同一诊断入口现在纯PC订阅，不再SSH/构建/启停SLAM；实际采集前后现有节点名单一致。
正式校时尚未执行、时间域问题尚未解决；需宇树确认无需SSH的固件/联网校时路径。
实验说明及测量：experiments.md最新节、analysis/go2_time_sync_20261112/pc_dds_lidar_state_summary.json。

## 2026-11-12 实机时间链诊断与未完成的源时钟同步

来源：远端a8f7c667、两轮30秒实采、Point-LIO内部只读日志及修正后阻断验证。
首个完整点云接收时刻被当作扫描起点对时，实测扫描跨度中位63.761ms，源DDS发布
相对header中位66.373ms；错误偏移最早污染转换后IMU（中位超前75.580ms）。
Point-LIO扫描末端按点内时间只加一次；内部end/odom一致到浮点精度，机身节点
446/446条完整保留stamp，精确源时间TF查询446/446成功。基线base_future中位
57.215ms、P95 67.361ms、最大71.510ms；不能把时钟问题写成定位漂移。

已删除首帧接收推算offset，使用原生时间/显式独立测量offset，首帧时基验证前不发IMU。
原始时间域仍与PC差约3,390,496s：源主控无法登录、NTP无响应，无可验证校准基准。
修正后30秒原始cloud/IMU连续，但转换和里程计均被阻断，**不能声称有效里程计已修好**。
定位算法/滤波器/外参/MAP接近参数/0.10s保护均未改；只增加默认关闭的内部时间日志。
仅构建必要的Point-LIO/转换包；无运动、无仿真、无批量单元测试。入口：
`bash camera_ws/scripts/run_go2_time_diagnostic.sh`。完整运动入口也先执行此诊断，未通过不使能。
证据与未验证项见experiments.md最新节和analysis/go2_time_sync_20261112。

以下为有日期的历史记录，不覆盖上述当前时钟阻塞状态。

2026-11-12 当前代码更新：MAP抓取停靠改为机身误差vx/vy/wz闭环，完成门限4cm/10°并静止0.5s；推荐S1矩形只记录参考。只取消抓取停靠固定总时限，保留ODOM25s/5m等保护，新增10s无进展停止。66项定向测试、40个普通MAP/SEARCH兼容快照、单包构建通过。三自由度横移控制尚未实机验证，本轮无硬件连接或运动。详见camera_ws/docs/map_s1_grasp_integration.md及experiments.md最新记录。

2026-11-12 文档更新：仓库根目录新增 `GO2实机测试命令手册.md`，集中列出本机手动测试全流程与稳定MAP回退命令；40个bash命令块通过语法检查。本轮只整理文档，没有重新构建或操作机器人；既有停靠失败及完整任务未验收的结论不变。

更新日期：2026-10-06
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

### P0 ROS 2 interface baseline (static)

- 已完成 24 个仓库内 ROS 2 package 的静态清单，并按主链、辅助、仿真和旧/替代流程分类。
- 已从 `nav_launch.sh` 及其 launch include 树确认主链 node、关键 topic、消息类型、参数来源和 remap；详见 `docs/interface_inventory.md`。
- 主数据流第 1 至 8 段均可由仓库源码静态闭合；safety gate 到 GO2 的第 9 段只能确认本地 publisher，GO2 外部 subscriber 和物理执行仍为 `NOT VERIFIED`。
- 静态 TF 意图为 `map -> camera_init -> aft_mapped -> sensor -> vehicle`，并有 `sensor -> camera`；Point-LIO 发布 `camera_init -> aft_mapped` 动态 TF。
- `transform_sensors` 内的 `body -> utlidar_*` TransformStamped 没有广播，不能作为实际 TF edge。
- 本轮没有启动 ROS graph，实际节点名、QoS 协商、TF 连通性与消息频率均为 `TODO: runtime verification required`。

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
- `transform_sensors` 从用户 Desktop 读取 `imu_calib_data.yaml`，文件缺失时使用源码默认值；当前真实标定来源和有效性尚未确认。
- 仓库包含旧流程、当前 DDS 流程、仿真和实验脚本；package 已完成静态分类，但 shell 脚本的正式支持/弃用清单仍未建立。
- Point-LIO 配置使用 `use_sim_time: true`，FAR launch 显式设置 `false`，其余主链节点通常使用 wall time；真实 launch 未启动 `/clock` publisher。
- `mapping_utlidar.launch` 将顶层 `use_imu_as_input` 设为 `false`，与 `utlidar.yaml` 中的 `true` 不一致。
- `go2_keyboard_teleop` 与 `transform_sensors` 的 manifest/setup 未声明源码实际使用的全部运行依赖，干净环境 rosdep 完整性待验证。
- Point-LIO 与 local planner 都声明 `/path` publisher；当前 Point-LIO YAML 通过 `path_en=false` 禁止其 path 数据输出。

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
- 本轮静态分类已给出 24 个 package 的主链/辅助/仿真/旧或替代流程角色；仍需运行时确认混合用途 package 的实际启用组件。
- 实时运行时 `/clock` 是否存在，Point-LIO 的 `use_sim_time` 是否与主线一致？
- GO2 当前固件、ROS domain、CycloneDDS QoS 和网卡配置的正式支持矩阵是什么？
- `imu_calib_data.yaml` 的权威版本、生成方法和部署位置是什么？
- 主链真实 TF 树及各点云/里程计 frame 是否一致？
- README 的 retest 指标如何计算，分析脚本与原始日志能否复现这些数值？
- safety gate 的 Sport lease、ARMED 和 API 成功能否对应到可测量的真实运动？

## Standalone D435 Perception Workstream

- 独立工作空间：`camera_ws/`；不加入现有 `code/` 导航 build/launch。
- ROS 2 package：`go2_science_perception`，提供只读 camera diagnostics 和 CameraInfo inspector。
- 2026-09-28 宿主只读枚举确认 USB `8086:0b07` D435 和 UVC device nodes 存在。
- 2026-09-28 已从 ROS 2 Ubuntu Jammy 官方 APT 仓库安装 librealsense 2.58.4 与 RealSense ROS wrapper/description 4.58.4；D435 firmware `5.12.7.150`，USB 3.2。
- 官方 wrapper 和 `camera_ws` 完整 launch 均已实测 RGB、raw depth、aligned depth、CameraInfo 与相机内部 `/tf_static` 数据；640x480 color/aligned-depth 尺寸一致，实际 frame 已记录在 `camera_ws/docs/architecture.md`。
- `camera_ws` 独立 build 成功，4 个 package tests 全部通过，完整 launch 运行超过 30 秒并干净退出。CameraInfo/metadata 约 30 Hz，但 Python 大图像订阅约 RGB 18 Hz、aligned depth 8 Hz且有短暂 freshness 告警，持续端到端 30 Hz 尚未验证。
- camera optical -> GO2 vehicle -> SLAM map 的只读TF2转换原型已实现；外参仍为 `PROVISIONAL / NOT CALIBRATED`，且未接导航。
- 独立 `blue_surface_center` 已回退会稳定选择侧面的上部55% ROI，改为完整蓝色target mask点云、最多4平面顺序RANSAC；先执行35°法向硬过滤，再按内点数和残差选择。30秒实测204帧、56 VALID（27.5%）；主要拒绝为 `bad_normal=118`，其次 `low_inlier_ratio=19`。实测存在最大侧面误差85.860°而第二候选误差11.070°并被选择的帧，证明最大平面没有被直接当作上表面；当前比例仍不能称为稳定。输出 frame 为 `camera_color_optical_frame`，尚未做 GO2/map 转换。
- 2026-09-29 已加入基于完整mask有效深度中位数的距离/尺度自适应：`<1.0 m -> 0.008 m`、`1.0–1.5 m -> 0.012 m`、`>=1.5 m -> 0.018 m`；RANSAC最少内点为 `max(30, ceil(point_count * 0.03))`。`perception_quality` 综合 mask面积、有效深度比例、平面内点比例和法向误差，只有 `GOOD` 发布精确相机系3D中心，`MARGINAL/UNRELIABLE` 仅保留2D/debug观测。
- 同日完成三档各15秒受控实测：近档0.284–0.286 m为49/49 `UNRELIABLE`（全部 `bad_normal`）；中档1.194–1.215 m为62/63 `MARGINAL`、1/63 `UNRELIABLE`；远档1.795–1.877 m为63/67 `MARGINAL`、4/67 `UNRELIABLE`。三档均无 `GOOD`，因此均未发布精确3D中心。该结果证明分段阈值和质量门控在实机数据上生效，不证明精确3D识别已稳定。
- 当前源码default和运行YAML的 `mount_pitch_deg` 已统一为30.0°。上述三档数据采集时仍使用45.0°，当前30°配置的近/中/远表现为 `NOT VERIFIED`。
- 独立 `coarse_target_locator` 已扩展为多蓝色目标：HSV/aligned-depth候选先以3D RANSAC剔除大范围高支持竖直蓝墙，再对剩余区域按2D连通与深度层聚类；每目标在既有topic连续发布一个PointStamped。2026-10-02实机静态验证最终窗口中，仅蓝墙为0发布，蓝墙+单目标稳定为1，蓝墙+3个不同大小目标稳定为3；不接导航，且同色目标与墙近乎共面时仍可能不可分。
- 2026-10-03针对蓝墙残片补充全平面距离过滤：RANSAC仍负责拟合，拟合成功后以
  `wall_rejection_band_m=0.08 m`删除全部近墙蓝色有效深度点，并以参数化
  `target_min_wall_separation_m=0.10 m`检查剩余cluster median墙距。固定3目标+1蓝墙
  的35秒稳定窗口为134帧×3点；完整链另一个35秒窗口为160帧×3点，manager只确认
  T3/T4/T5且未继续增长。启动期仍创建过两个未确认瞬态track，近共面真实目标边界
  尚未验证。
- 2026-09-29完成0.5/1.0/1.5/2.0 m四档实机粗定位：实际median Z分别为0.487–0.488、0.941–0.954、1.467–1.513、1.918–1.998 m；发布统计37/37、60/61、38/38、64/64，frame均为 `camera_color_optical_frame`。这是功能与连续发布证据，不是外部真值测距精度标定。
- 新增独立 `camera_target_to_map`，按输入PointStamped原始时间戳查询TF2，发布 `/science/target_coarse_point_map` 与RViz Marker。暂定 `vehicle -> camera_link` 外参为 `(0.36,0,0.12) m`、RPY `(0,+30,0) deg`，并复用RealSense的camera_link到optical TF；`coarse_target_locator`未修改。
- 只读实机组合明确未启动FAR/WP5/local planner/path follower。有效A/B试验中蓝色目标静止、GO2由用户人工移动，map目标均值三维差约14.9 mm（25与32样本）；整轮2347次转换、TF失败0。该单次结果支持“本场景基本稳定”，但外参仍为 `PROVISIONAL / NOT CALIBRATED`，不能作为导航接入或标定精度证据。
- 2026-10-02在固定目标下完成名义正面/左/右平移组及同一位置中间/左转/右转航向组复测。平移组位置均值最大XY差为94.0 mm，旋转组航向均值最大XY差为26.9 mm；人工摆位未用外部量具验证，且map转换存在3262次时间戳TF失败，仅成功发布样本进入统计。该结果表明暂定外参/map链仍有明显视角相关偏差，不支持导航接入；完整数表和边界见 `camera_ws/docs/experiments.md`。
- 同日完成TF future-extrapolation专项：旧日志372条可解析样本的`point-latest TF`差为0.000229–6.590080 s。D435实机launch现显式设置Point-LIO `use_sim_time=false`，map转换对原始点时间戳执行最多8 s有界重试并逐次记录point/latest/delta，绝不替换为latest TF。固定目标五视角复测内部计数为3579/3579转换成功、0最终失败（1次启动期miss随后精确时间重试成功）；该结果不改变外参仍为`PROVISIONAL / NOT CALIBRATED`及暂不接导航的结论。
- 本工作流仅包含蓝色区域测试检测，不包含通用目标分类、导航接入或 GO2 运动控制。
- 2026-10-02新增`science_target_manager`导航侧第一阶段离线实现：按同帧多点做
  map XY一对一关联（默认0.20 m），5帧确认，近期median位置和稳定ID；基于经TF
  转到map的`/base_state_estimation`选择最近未访问目标并生成0.40 m standoff可视化。
  confirmed/selected/standoff均有RViz输出，但没有`/goal_point`、FAR/WP5或运动命令
  接口。源码确认`/base_state_estimation`继承Point-LIO的`camera_init` header，因此
  管理器不会直接将其当作map。单元/静态验证已完成；真实ROS graph、连续TF和真机
  多目标Marker运行链已于2026-10-03完成约2分钟只读实机验证：RViz同时收到多个
  confirmed、selected、standoff及visited重选更新，未接`/goal_point`或运动链。
  现场候选较多并累积到至少T18，因此每个稳定ID是否都对应独立真实目标仍为
  `NOT VERIFIED`。
- 2026-10-03在3个固定真实目标、GO2人工前后/左右/原地转向/斜移条件下完成8个
  15秒稳定窗口。原始T1–T3的最大跨阶段均值XY差为34.62 mm，存在厘米级视角相关
  偏差；旋转/斜移又产生T6/T11/T14/T17，最终7个confirmed ID对应3个真值，且
  selected转向额外track T14。standoff几何仍保持距selected 0.40 m，但目标语义不再
  可靠；当前结论仍为不得接入FAR或运动控制。
- 2026-10-06仅为`science_target_manager`增加candidate/confirmed/stale生命周期、
  超时删除、连续多帧confirmed合并和active-confirmed-only选择；关联半径仍为0.20 m，
  FAR/WP5/规划/控制接口均未接入。48项package tests通过且独立build成功。三固定目标
  的8阶段人工移动重跑中，每个15秒稳定窗口均收敛为3个ACTIVE CONFIRMED，selected
  未引用STALE/已删除ID，standoff持续0.400 m；三个空间目标跨阶段均值最大XY差为
  54.02–64.28 mm。实机发生STALE/REMOVED但未触发MERGED，因此合并实机触发仍为
  `NOT VERIFIED`；完整证据见`camera_ws/docs/experiments.md`。外参仍为
  `PROVISIONAL / NOT CALIBRATED`，仍不得接FAR或运动控制。
- 2026-10-08将manager完成语义改为实时track加map空间访问记忆：新增
  `visited_positions`、默认`visited_radius=0.15 m`及
  `/science/mark_selected_visited` Empty触发；新track ID只要位于visited半径内仍被
  selected排除。54项package tests和隔离build通过；固定3目标的`3 -> 2 -> 1 -> 0`
  真机验收因`enp12s0 NO-CARRIER`、无GO2 map输入而为`NOT VERIFIED`。D435仅确认USB
  枚举存在。蓝色检测、蓝墙过滤、camera_to_map及无运动边界未改。
- 2026-11-03完成3个固定目标实机复测。用户接受视野变化导致目标短暂刷新；可见稳定
  窗口中map变换连续且standoff为0.400–0.402 m。visited人工触发完成
  `3 -> 2 -> 1 -> 0`；原T2遮挡后以新ID T70重新确认，并在距历史visited位置
  0.028 m处继续显示`visited/ignored`，证明过滤不依赖track ID。遮挡和启动瞬间仍会
  产生蓝色误检，属于已知限制。
- 同日增加最小一次性`science_goal_bridge`，只接受有限`map` frame standoff并锁定首个
  目标转发到`/goal_point`。56项package tests和独立build通过。FAR/terrain/localPlanner/
  pathFollower在不启动WP5、安全门、武装或Sport输出的条件下启动；FAR自带RViz因注册
  第二个`/goal_point` publisher而被停止。启动污染轮的一次goal发布已作废，干净轮因
  额外T6确认并抢占selected而在bridge启动前停止。因此有效FAR收目标、`/way_point`、
  `/path`及`/cmd_vel_raw`均仍为`NOT VERIFIED`。

## Next Recommended Work

仅建议调查，不在本轮实施：

1. 在无真机条件下建立 package、node、topic、frame 和参数的完整静态清单。
2. 在干净 ROS 2 Humble 环境执行依赖检查与可重复 build，记录精确命令和失败项。
3. 分类现有测试：纯静态/单元、合成数据、依赖 rosbag、依赖 GO2 真机。
4. 使用一组明确来源的 rosbag 做只读离线回放，验证 Point-LIO 到 `/cmd_vel_raw`，不启动真实运动输出。
5. 完成硬件安全清单后，再设计分阶段 dry-run 和低速真机验证。

## 2026-11-06 — 独立单目标接近（本轮授权范围）

- 分支 `feature/d435-perception-workspace` 新增 `science_target_approach`；独立预览
  launch强制dry-run，既有视觉/蓝墙/Point-LIO/FAR/WP5算法不改。manager仅新增带原始
  观测时间、track寿命、ID及visited的只读JSON快照，供稳定锁定和可见性判断。
- D435已移至GO2主机，Foxy独立相机驱动构建完成，经用户对时、限定DDS peer和话题
  remap后PC实收RGB约14.9 Hz、aligned depth约14.4 Hz。源事实与实测见
  `camera_ws/docs/experiments.md`；外参仍 PROVISIONAL / NOT CALIBRATED。
- 初轮dry-run被蓝色误检抢占nearest；用户后续将检测参数调为s_min=110、
  min_component_area_px=250，Codex同步YAML并重编译，155/155及151/151帧各只有1目标。
  随后T1稳定锁定、预览最高0.30、检测/定位输入超时锁存均通过。
- 默认0.80 m停靠、最高0.30 m/s、轨迹寿命门槛、明确ID确认、TF/定位/状态/竞争发布者
  检查、终态锁存和JSONL日志已实现；配置关闭gate的0.20 m/s最低速度抬升。
  默认保持预览；真实模式仅在用户后续逐次明确授权下经gate输出。
- 用户随后明确授权0.10 m/s短时及0.30 m/s上限完整接近。完整轮锁定T1，最高Move命令
  0.294 m/s，约5.73 s、距离0.917 m时视觉丢失并停车，后续里程计距离0.875 m。
  用户确认方向正确、已停住、无明显异常；这不是0.80 m定距停车通过。窗口位姿差分
  速度峰值0.494 m/s（真实性待核），实际速度不超过0.30 m/s仍NOT VERIFIED。
- 真实控制器和gate已停止、raw publisher=0；相机/SLAM/检测链按用户要求保留。
  后续定距停车验收及阶段三仍未完成，APPROACH_ODOM真实模式继续由代码拒绝。

## 2026-11-06 — MAP锚点与相机原点接近（858f75a后续）

- 已实现稳定中位数MAP锚点、按空间位置跨ID重关联及小幅视觉修正；视觉间断不删除
  锚点，APPROACH_ODOM在实时Point-LIO/相机TF有效时闭环，默认25 s/2.0 m上限。
- 停车参考改为相机camera_link原点的MAP XY，真实带时间戳TF包含旋转后的安装偏移；
  0.10 m容差，忽略Z。默认预览不发raw，显式--execute入口自动录像/锁定后执行一次。
- 旧轮0.917 m视觉FAULT主要与0.38 s输出间隔叠加0.5 s时间门槛有关，不支持直接判定
  出视野/面积不足/depth失效。HSV/墙过滤算法未改，增加原检测轮廓、mask和诊断输出。
- 本轮第一轮kp=0.5于35 s超时，MAP误差34.2 cm，用户实测35 cm。kp提高至1.0后，
  一轮在7.827 s进入camera XY STOP，误差8.29 cm，最后9.43 cm；用户确认约10 cm处
  停住且效果良好。只记录本场景粗验收通过，正式标定/重复精度仍NOT VERIFIED。
- 实际RGB检测MP4已保存，成功文件为camera_ws/records/approach_20261106_210841_496906.mp4，
  同名JSONL含逐帧、控制及用户实测数据；完整204/204帧解码通过，数据本机保留不提交Git。
- 指令最大0.30，Sport反馈峰值0.316 m/s，不能把指令限速写成实际速度严格上限证明。
  结束撤下实机controller/gate、raw publisher=0；保留只读感知及独立日志窗口。

## 2026-11-07 — 移动搜索与自动MAP接近

- 在原`science_target_approach`内增加SEARCH，沿启动MAP朝向最多1 m，初始0.15、
  用户后续要求改为0.20 m/s；一个raw发布者切换到原接近控制，不启动FAR/WP5/局部规划。
- 最近1 s至少5个不同原始时间戳MAP观测、最大两点XY散布<0.10 m自动锁定中位数锚点，
  无3 s寿命等待、不依赖像素静止；保持原视觉空间修正、失视觉Point-LIO闭环和急停。
- 实机A为18 cm用户实测超时失败；B为ALIGN超时；C为未确认目标搜索超时。
  D使用0.20搜索、kp_linear=1.5、kp_angular=2.0、braking_latency=0.30，自动切换后
  9.800 s以camera XY=0.0927 m触发到达STOP，用户确认“接近10cm完成”。
  最后MAP读数0.1130 m，Sport反馈峰值0.3503 m/s，不能宣称精密停车或实际严格限速通过。
- 实际检测MP4完整241/241帧：`camera_ws/records/approach_20261107_172248_186678.mp4`，
  同名日志及run摘要本机保留；详细失败/成功证据见`camera_ws/docs/experiments.md`。
- 独立构建、24项针对性控制检查通过。暗光识别使用用户已实测结论，不重复测试、不调HSV。
  外参仍PROVISIONAL，重复精度/实际严格速度上限/路线终点物理停车仍NOT VERIFIED。

## 2026-11-07 — 重复搜索失败调查与会话清理

用户重复轮`run_1794043913372908704`末段确有RGB检测，但控制锚点未恢复视觉更新。
原锚点(3.697,0.694)m，后续原始MAP点偏移数十厘米并产生T89/T90，超出原空间门槛。
接近8.800s、失视觉累计里程计路径2.003m时触发`odometry continuation limit`，当时
相机XY误差1.113m，不是到达停车。MAP偏移根因仍待查；未调HSV或放宽门槛。
已按用户要求停止PC只读链及远端D435，原始332帧视频/日志保留。详情见camera_ws实验记录。

## 2026-11-08 — 固定物块 MAP 漂移诊断准备

基于9d657b8，新增观测记录（原始RGB/Depth stamp、camera XYZ、实际MAP TF、
关联分支原因）、只读JSONL采集与统计脚本；不修改HSV、Point-LIO、关联门槛或控制计算。
旧录像332帧完整解码，175检测/80有效帧实际时差绝对P95约67.19ms；旧逐帧MAP/TF/
Point-LIO原始姿态不足，且/tmp感知日志已丢失，底层根因仍Unknown。
A/B2/C诊断数据已采集：A最大/P95 2.51/2.27cm；B2物块注释14.00/4.07cm；
C候选13.72/5.96cm（最大点身份被录像tile遮挡，NOT VERIFIED）。严格B/C运动和全程
可见性未达，C用户只部分返回。未复现旧23–39cm已核实同物块漂移，根因仍Unknown。
GO2时钟已恢复，正式采集前同连接SSH测得约-6.1ms。TF算术一致不证明外参或定位准确。
详见camera_ws/docs/map_drift_diagnostics.md。不得将本次时钟异常反推为前轮根因。

## 2026-11-08 — 5cm目标STOP_CONFIRM流程（已实现，6秒版待测）

基于a8fb268新增SEARCH→STOP_CONFIRM→原APPROACH，源匹配相机距离≤1.8m连续候选
触发停车，反馈/位姿停稳后仅收新样本，至少5帧/0.6秒源跨度、散布<0.08m取中位锚点。
用户最后指定确认总时限6秒，包含制动/停稳/采样，冷却0.75秒恢复原有界搜索；其余
HSV、Point-LIO、关联/ODOM保护和原接近控制不变。30项相关检查、package构建及lint通过。
已完成4轮真实运行：初始可见2秒版用户实测9cm成功；移远后有前端无候选失败；2秒版
搜索发现但停稳超时；4秒版两次确认超时后按搜索总时限停车。严格不可见起步的重复
成功率尚未证明提高。6秒版按用户要求今天不再运动，明天实测，NOT VERIFIED。
D435先前未枚举问题重插后恢复；曾出现PC USB过流日志，连接状态需明天复核。
上一轮中断的10mm分析未纳入本轮，备份在/tmp/go2_10mm_pending_before_stop_confirm_20261108。

## 2026-11-09 — 用户指定远目标搜索参数与复测

SEARCH配置/执行入口默认改为5 m、60 s，速度由0.20改为用户确认的0.25 m/s；
确认总限保持6 s，SEARCH专用ODOM路径上限5 m、失视觉时间25 s，安全门仍0.30 m/s。
31项针对性检查、独立package build、修改Python文件flake8与diff检查通过。
USB实收D435；GO2时钟退回2022年，经用户本地输入sudo密码恢复后才启动传感器。
第一轮仍用1 m搜索，未获有效候选，路线结束，用户确认停稳，双录像237/237帧解码。
第二轮按5 m/0.25运行，进度约3.413 m时段后定位超时；用户报告中途切换遥控器并
确认停稳。同段网卡carrier-changed/DOWN、DDS写失败；不将网络事件全部归因遥控器。
该中断轮未进入STOP_CONFIRM，非6秒确认验收；双录像327/327帧完整，runner收尾
中断未生成run摘要，另存明确标注的post-hoc audit。恢复链路后用户另授权重试。
完整结果/原始证据见camera_ws/docs/experiments.md；5 m边界与6秒完整流程尚不可
仅据参数和构建认定通过。未修改Point-LIO、HSV、空间关联或安全门。

恢复链路后第3轮第一次6秒确认超时，第二次总耗时3.400 s确认成功（停稳1.900 s、
采样1.500 s、6帧、散布5.26 cm），随后原接近约10.00 s到达STOP。首次MAP XY误差
7.59 cm、收尾8.80 cm；用户明确确认停稳并实测9.5 cm。两份录像均560/560帧完整。
本场景6秒确认/接近链取得一次用户确认结果；未行满5 m搜索或5 m ODOM边界，
这两项真实边界与重复精度仍NOT VERIFIED，不能外推成功率提升。

后续用户报告当前版本“实测两次都可以”，并要求推送。已记录为用户现场反馈；所述
两次未逐一关联原始run并审计，不补写距离/耗时或定量重复精度。实验实现为07292cd，
参数保持搜索5 m/0.25 m/s/60 s、确认6 s、SEARCH专用ODOM路径5 m/25 s。

## 2026-11-12 — 单目标 MAP/S1 集成实验分支

基于远端稳定 e15c66c，新模式默认关闭：右侧抓取停靠二维几何、MAP 局部停靠、
SDK2 姿态交接及原 S1 socket 任务协调已实现。普通 MAP 参数未改。
43项初始定向测试通过，单包构建通过；末端切换修复后44项定向测试通过。
GO2 已接 D435/D455、S1串口，交付包离线预检通过；S1第二次启动进入 ZERO_HOLD。
首次真实停靠试验 SEARCH/STOP_CONFIRM成功，末端阶段切换后run timeout，未趴下/抓取。
新增最终航向阶段锁定及先制动修复；修复后实机结果尚待记录，不得把本次失败写成完整抓取成功。
实际XY测量、趴下姿态完成判据、StandDown/grasp/StandUp均 NOT VERIFIED。
详见 camera_ws/docs/map_s1_grasp_integration.md 和 camera_ws/docs/experiments.md。

### 2026-11-12 姿态实测及完整入口后续

已实测并写入本机姿态配置：down<=0.084m/up>=0.304m，mode均0；
StandDown/StandUp各实际完成一次，趴下稳定由用户确认。
SDK2默认加载DDS库会段错误；仅给姿态子进程指定编译匹配的0.10.2库后租约正常。
完整入口一次因定位失稳在SEARCH前拒绝；原参数重启定位后，SEARCH/STOP_CONFIRM成功，
停靠仍25秒超时，最终MAP位置误差9.94cm，预测S1 XY未进入推荐区域；未请求抓取。
本次最近距离3.16cm，未进入2cm完成容差；上一轮最终航向阶段修复未解决全部末端收敛问题。
参数已填并不代表一键任务实机通过。当前不能作为已验收的机械臂自动抓取版本。

2026-11-13 最新用户修正：横向额外净距改为10cm，替代前述27.5cm；S1右侧停车目标0.41m（原0.31m+0.10m）。前后位置/容差/速度不变；仍先dock实测，抓取可达性NOT VERIFIED，本轮未运动。

## 2026-11-13 微调最低10cm/s，取消停靠25秒时间上限

用户明确要求最低平移微调10cm/s，并取消触发微调后的25秒限制。
集成抓取停靠profile新增dock_min_linear_speed=0.10，作用于4cm容差外的合平移目标速度；
不是vx/vy各10cm/s。制动限速、实测速度制动、向量加减速和0.30m/s上限仍优先，
故最终指令可低于0.10m/s；进入容差后降到0并沿用10°/0.5s完成条件。
抓取停靠不再使用丢视觉25秒预算及基于剩余时间的拒绝，遥测remaining_time=null；
保留5m续行距离、剩余距离判据、定位/TF/反馈新鲜度、异常停车、人工急停、唯一控制出口。
SEARCH原5m/60秒及普通MAP时间逻辑不变；横向额外10cm保留，前移3cm尚未实施。
34项停靠/接口检查通过，覆盖10cm/s合速度/制动及25秒后继续、5m仍停车；无离线仿真。
本轮未实机运动，新速度和长时间丢视觉停靠实机NOT VERIFIED，实际可达性不据软件检查推断。

## 2026-11-13 SEARCH速度0.35m/s（用户指定）

SEARCH profile从0.25改0.35m/s，SEARCH独立上限校验改0.35；共享安全门max_speed改0.35以免截断。
接近/停靠控制器max_linear_speed仍0.30，微调最低目标0.10且制动优先，角速度及加减速不变。
SEARCH仍5m/60秒，停稳确认/唯一控制出口/定位异常停车不变；前移3cm尚未实施。
本轮仅软件修改，不启动运动，新速度真实行走与停止表现NOT VERIFIED。

2026-11-13 实机启动拒绝修复：0.35m/s SEARCH下安全门已配0.35，但审计仍以接近0.30判定导致gate not verified；仅审计上限取已启用SEARCH与接近上限的最大值，普通MAP仍0.30，保留其余唯一出口/租约/制动校验。此前启动被拒绝，未运动。

2026-11-13 用户要求恢复SEARCH速度：实际集成profile search_speed恢复0.25m/s，安全门max_speed恢复0.30m/s；微调最低0.10、停靠无25s时间上限及横向额外10cm保留。S1最近串口I/O错误，本轮重试仍保留硬件阻断。
