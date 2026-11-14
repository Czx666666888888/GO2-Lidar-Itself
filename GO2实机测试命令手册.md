# GO2 实机测试命令手册（中文，从接线到关闭）

更新：2026-11-13。适用本机 PC + GO2 + D435 + S1 + 腕部 D455。
已增加下面的一键入口；后续章节保留逐步操作和故障恢复说明。

## 2026-11-14 精调请求下限0.18m/s（当前）

用户后续要求4～15cm修正请求下限提高为0.18m/s，旧0.08m/s上限同步移除，
精调上限沿用0.30m/s总上限；15cm外下限仍0.20。制动、实测速度和加减速限制优先，
所以6cm附近按原制动公式仍可降至约0.089m/s，不能将请求下限写成实际速度保证。
≤4cm的零速等待与原成功条件不变，等待期内即使误差4～6cm也继续原等待逻辑。
以下“精调和停车等待”节的0.08速度为历史版本，仅其等待规则继续适用。
本次仅离线测试/构建，无运动；实机效果NOT VERIFIED。

## 2026-11-14 停靠精调和停车等待

最后15cm改为精调，请求平面合速度上限0.08m/s；远段0.20m/s下限、总上限0.30m/s保留。
实际过渡仍按原制动/加减速限制。误差≤4cm、yaw≤10°后持续请求零速，gate按原限制减速。
日志docking_phase为STOP_WAIT；4～6cm内小波动继续等待，不能据此通过到位。
>6cm或yaw>10°恢复修正；在4cm外等待2秒也恢复精调。成功仍要求≤4cm/10°与原速度条件
连续0.5秒，完成后才允许原趴下交接。没有放宽失视觉5m预算、故障保护或实际抓取条件。
本次只完成离线检查及构建准备，无真机动作。先另行授权独立dock验证，真实停稳效果NOT VERIFIED。

## 2026-11-14 S1串口0/1兼容

一键入口自动枚举`/dev/ttyUSB0`和`/dev/ttyUSB1`，仅一个存在时传给原S1服务。
两个都在或均不存在会停止，不按序猜选；该枚举不证明物理设备身份，原SDK/反馈预检仍保留。
已有服务仍绑定其他端口时保留服务并阻断，不自动停保持/重新使能，也不创建第二串口所有者。
需先在机械臂有支撑、任务已停止的条件下处理旧服务，再重新启动。本次仅离线检查，
未重启远端服务或运动；实机串口恢复NOT VERIFIED。正常测试命令仍为`bash go2_test.sh --execute`。

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

## 2026-11-13 取消停靠无进展退出，接入自主搜索/抓取/起立全流程

### 本次真实运动与用户反馈（修改前）

本轮仅发生一次真实运动。第一次准备因末端无新鲜CONFIRMED目标而退出，没有运动；
同时诊断timesync-status查询可能经D-Bus激活timesyncd。a3de41b已跳过受控期间该查询，
并增加服务意外重启时请求原任务停止的监测，恢复原active状态后不再继续。
更正：早先“受控暂停”记录仅证明任务前停用/结束恢复，未监测全程，不能据此证明全程暂停或NTP因果。
数据采集通过的事实仍保留；实际服务重启记录使此前全程暂停的表述需要降级。

用户重新放置目标并确认继续后，full_1794568140327183944/pre_motion通过，进入一次dock运动：
SEARCH→STOP_CONFIRM（约3秒确认，6个样本）→APPROACH_TO_GRASP_POSE→FAULT。
故障为grasp docking stalled: no meaningful pose/stop progress，elapsed19.600s。
最终位置误差129.272mm、航向误差约7.54°，末端命令vx/vy/wz均0，反馈速度约0.00354m/s。
位置误差曾短暂到35.360mm，但未满足全部停稳完成条件，不能算停靠成功。
用户最终现场反馈：“停稳定了但是位置有所偏差”，以此作为物理结果，不将先前“基本正确”写成精度验收。
没有趴下、抓取或起立，任务未自动重试。运动/定位进程已退出，PC校时恢复active，S1 ZERO_HOLD空闲。
双录像按writer记录各376帧并closed，保留本地；未在本轮逐帧复核视频。
证据：camera_ws/records/run_1794568209143452018.json及science_approach_1794568210039015787.jsonl；
摘要camera_ws/analysis/go2_dock_stall_20261113/summary.json。

### 按最新要求的最小静态修改

1. 只移除GraspDocking中“超过progress_timeout即FAULT”的分支。
   累计进展与误差继续记录用于诊断，不参与停止决定，状态带docking_progress_timeout_enabled=false。
   旧dock_progress_timeout配置仍接受以保持兼容，但不再用于退出。
   原指标只看超出位姿容差部分，曾进入容差后best_error可为0，无法再改善0.25；
   不能把这种“没有刷新历史最佳值”等同于控制器应停止修正。
2. 三自由度闭环、4cm/10°/0.5秒停稳完成条件、速度/加减速与唯一运动出口均不改。
   定位/TF/反馈新鲜度、定位异常、人工急停、S1保持心跳与安全门继续有效。
   **视觉丢失后的原25秒/5米ODOM预算仍会停止**；“继续修正”不表示忽略传感器失效或无限盲走。
3. PC时间上下文支持dock/full，go2_test.sh所有PC --execute均先进入暂停/恢复上下文；
   full传递给原go2_full_test/run_map_s1_task协调器，保持原姿态配置和S1动作链。
   不改S1抓取算法；停靠未完成时禁止StandDown/抓取，抓取失败不自动StandUp。

### 唯一全流程命令（真实运动、趴下、抓取、起立）

```bash
bash go2_test.sh --execute --mode full
```

该命令会进行环境/硬件/时间/定位检查、S1 ZERO_HOLD、D435/TF准备，随后自主SEARCH（原5米/60秒）
→STOP_CONFIRM→MAP三自由度抓取停靠→达到4cm/10°且停稳0.5秒→STOP并退出原控制器/安全门
→StandDown反馈确认→S1原完整抓取→确认新的成功结果并回ZERO_HOLD→StandUp反馈确认。
full运动前不要求物块已经在视野内，因此允许从无目标视野自主搜索；未找到目标仍按原SEARCH边界停止。
`--execute`不写mode时默认full；`--mode dock`仍只停靠且保留启动时目标MAP验收，不趴下抓取。
无--execute只打印计划，不启动硬件。Ctrl+C请求原停止；现场仍需急停可用和空旷路线。

修改后验证：30项停靠/接口检查、6项S1协调检查通过；包含无进展及0.1mm级输入修正超过30秒
仍继续控制，以及原ODOM预算、定位超时、人工STOP/FAULT置零、完成停稳条件的现有检查。
首次接口检查因未source ROS缺builtin_interfaces而未收集；source现有Humble底包后同范围30项通过。
单独go2_science_perception构建成功；脚本语法/计划打印通过，没有全仓构建或新运动。
**修改后的持续修正与full完整动作实机NOT VERIFIED**。本轮只做静态修改，不沿用前次运动授权。

## 2026-11-13 PC受控IMU定位通过，NX部署暂停（基线28ffcbf）

用户本轮改变优先级：只恢复PC运动条件，不推进NX容器/ROS迁移或完整S1抓取。
只读检查PC时钟源tsc、timesyncd active，NTP频率-500ppm、网络delay641ms/jitter517ms。
历史ROS与RAW60秒相差395ms的异常成立，但本次两轮20秒在线/暂停对照未复现：
第一轮wall-minus-RAW为-1.852/-3.955ms；第二轮为-1.613/-5.221ms。
因此**不能确认NTP是历史异常唯一原因**，也不能仅凭远端NTP高延迟推导本地时钟跳变。
CLOCK_REALTIME/MONOTONIC受频率校正、RAW不受同类校正，参考Linux官方接口文档：
https://man7.org/linux/man-pages/man2/clock_gettime.2.html
停止timesyncd不等于把内核频率修正归零；本轮未修改adjtimex频率、系统日期或GO2原装主控时钟。

### 实际受控执行与必要修复

新入口pc_time_check.py先原生预检/按需构建，再记录20秒在线时钟，sudo临时停止原本active的
systemd-timesyncd，记录20秒暂停时钟，运行当前会话IMU映射60秒；finally恢复原active状态。
不更改服务enabled配置，Ctrl+C/SIGTERM先等待原停止流程退出再恢复校时，异常结果返回失败码。
若恢复失败明确报错；人工恢复为sudo systemctl start systemd-timesyncd。
第一轮定位未启动：相机伴随主机仍偏差-107.413ms，原50ms检查阻断；PC服务已恢复成功。
已有sync_go2_clock.py在收到PC采样后再启动date子进程，会把该启动/调度延迟带入设时。
最小改为sudo授权后的同一Python进程调用clock_settime；仍只设置既有.18相机伴随主机，
不接触GO2原装原生雷达生产者主控。复验5次偏差+2.881～+2.972ms，启动检查通过。
该实测支持设时路径改善，但未测量date子进程全部延迟，不声称它解释所有历史时差。

### 第二轮60秒真实硬件验收

- mode=imu，原始IMU2秒窗口P5初始化共享b，未用首帧完整点云，未按帧覆盖now。
  b=3460654.208629460s；不复用上次offset；LiDAR/IMU共同使用，点内时间原样保留。
- 原生点云14.816Hz、中位丢包0.552%；889帧cloud及14982条原生IMU。
- Point-LIO/base各889条，机身最大接收间隔100.521ms；时间单调、位姿有限，最大相邻机身步长30.469mm。
- base stamp-minus-receive中位 **-23.421ms**，范围 **-59.911～-16.508ms**，即本窗口没有超前，
  已消除先前数百毫秒偏差，满足当前future_tolerance=150ms和odom_timeout=400ms。
- 机身TF889/889、相机TF893/893，均100%；目标MAP470/470精确源时间匹配，
  selected1359条，最终CONFIRMED未访问ID1且新鲜；用户确认蓝色目标已放好并保持可见。
- IMU窗口候选残差最终+29.262ms；接收-header趋势约+0.128ms/s，DDS source-header中位7.959us，
  暂无源端发布延迟放大的证据。接收差仍混合相对钟差与传输/调度，不能完全分离单向延迟。
  本次满足控制要求，**未增加在线offset修正**，更未提高任何阈值或修改Point-LIO/MAP/D435/S1算法。
- 总报告PASS，PC timesyncd已核对恢复active，S1只读ZERO_HOLD/busy=false/navigation_allowed=true。
  测试结束清理相机/定位；没有发运动、抓取或姿态命令。通过仅代表此窗口和单次dock测试数据条件，
  不代表长期定位精度或实际停车/抓取验收。

### 一键入口

只读复验：`bash go2_test.sh --time-diagnostic --pc-time-check`。
PC默认选择IMU模式；legacy保留显式可选，不再作为当前PC默认。
唯一单目标运动命令（由操作者在急停可用、路线空旷后执行）：

```bash
bash go2_test.sh --execute --mode dock
```

此命令自动进入同一受控校时暂停/恢复上下文，子流程通过GO2_PC_TIME_HELD防止递归。
整个dock退出前保持同样时间条件，退出后恢复原服务；不在只读验证后恢复NTP又直接套用测试结论。
复用原一键完整准备、原生预检、伴随主机对时、S1保持、D435/定位/TF/真实目标验收和唯一控制出口。
不会省略定位新鲜度/异常停车、SEARCH/STOP_CONFIRM或4cm/10°/0.5s及三自由度控制。
本轮没有执行这个运动分支，运动分支的新增上下文仅做语法/源码检查，实际运动仍NOT VERIFIED。
不启动NX工作，也不改full抓取动作链。

证据：camera_ws/analysis/go2_pc_controlled_20261113/；原始records下pc_time_1794566476733729043、
pc_time_1794566682116417530、time_20261113_184240_pc_paused_imu（启动失败）、
time_20261113_184550_pc_paused_imu（通过）。摘要含原始文件SHA256。
仅按需构建transform_sensors一次（上一轮主机时钟检测变更尚未安装），无批量测试/全仓构建。

## 2026-11-13 NX本机时间与入口适配（基线3c298c01970b）

实际NX：Ubuntu20.04.5/aarch64、Python3.8.10、ROS Foxy/Noetic，L4T35.3.1。
现有D435/Foxy与CycloneDDS包可找到；在实际source这两个overlay后仍缺Point-LIO、transform_sensors、
go2_keyboard_teleop和go2_science_perception四包。不能将PC的Humble/x86二进制复制到NX使用。
NX实际12秒纯订阅：点云179帧/14.883Hz、IMU2999条/249.819Hz，未启动额外雷达驱动。

### 时间处理及部署边界

转换器一直用运行节点get_clock().now()，此前运行在PC所以参考PC；现在本机入口使传感器和定位
都在NX运行，自然以NX ROS系统时间为基准，不是再加一个PC↔NX offset。
每次进程启动重新建立当前会话b，不读写持久offset；LiDAR/IMU共享b，点内时间不变。
保留legacy/imu选项，不把stamp逐条替换为now。legacy状态新增原始IMU接收差的实际变化量，
不靠在线修改b掩盖漂移。ROS与monotonic突变超过50ms，或源时间相对monotonic突变超过100ms时
停止转换输出并锁存故障；现有控制器400ms新鲜度/安全门超时负责请求停止。物理停止未实测。
恢复需停止控制并重新执行本机入口，新会话重建地图/offset；不在运动中自动重锁时间或地图。
未改Point-LIO、识别、SEARCH/STOP_CONFIRM、MAP管理、3DOF/停车条件或S1动作链。
use_sim_time默认false，组合launch明确false；诊断仍拒绝有/clock的异常环境。

新增GO2_RUNTIME_PROFILE=local与go2_test.sh --local-check，ARM64默认本机只读入口；
不调用PC SSH/校时，不配置PC peer。ROS setup、DDS/camera/导航overlay和网卡移到主机env文件，
样例config/go2_local.env.example。已有CYCLONEDDS_URI由部署者配置，无网卡配置则DDS自动选择。
诊断路由/接口采集不再硬编码PC地址/网卡；原PC环境脚本在local模式不再注入PC peer。
核心时间代码无/home/czx666、enp12s0、PC NTP或PC /clock依赖。
PC开发入口scripts/go2_full_test.py的SSH相机、对时和S1编排仍保留，不能当作NX完整任务入口。
NX的--execute明确阻断，避免回落PC路径。**本轮只完成本机只读入口，完整本机运动/S1编排待迁移验收。**

### 最小环境迁移与实际验证

Humble官方二进制目标Ubuntu22.04/Jammy（ARM64也有）：
https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/Ubuntu-Install-Debs.rst
优先保留NX宿主20.04与JetPack，在本机ARM64 Jammy/Humble容器内编译缺失包，DDS使用host网络，
USB只传入已确认相机设备；D435驱动需在同一容器/ROS环境编译，不能复用Foxy二进制。
容器镜像/USB权限/驱动兼容、SDK Python与ROS解释器隔离、实际构建和运行均待确认，未下载或安装。
另一条路径是在Foxy下源码构建四包，逐项适配rclpy MessageInfo执行器、launch/static TF参数、
RealSense profile参数及Python依赖；不改算法，但不能未经构建就宣称兼容。
不修改NX系统日期/NTP或重装系统；绝对日期与原装主控一致不是前提。

已把独立入口部署到NX的/home/unitree/go2_host_local，实际执行返回exit2并明确列出四个缺包。
没有启动Point-LIO、D435或TF链，因此持续定位/相机MAP/完整自主运行均NOT VERIFIED。
本轮只有原生DDS是NX实测，不使用PC以往结果代替NX验收。无全仓构建、批量测试或运动。
证据camera_ws/analysis/go2_nx_local_20261113/environment.txt；脚本语法/Python编译检查通过。

NX唯一只读入口（从部署目录执行；当前会明确报缺包）：

```bash
bash go2_test.sh --local-check
```

PC仅可通过SSH执行同一NX入口。迁移完成后同一入口才会启动本机映射/Point-LIO/base/TF/D435/MAP
并输出结果；它不依赖PC在线。当前不能提供已验收的NX完整运动启动命令。

## 2026-11-13 18:13 legacy一键真实验证：流连续，时间/末端目标验收失败

源码基线b3b603a35e9b；执行`bash go2_test.sh --time-diagnostic --legacy-check --duration 60`。
原生预检14.662Hz、丢包中位1.657%，通过；复用已构建包，没有重新构建。
用户在本地终端输入sudo密码，现有脚本对时后5次伴随主机偏差-15.600～-15.542ms，原检查通过。
日志实际显示TIME_MODE=legacy、first_cloud_offset_ns=3460654380674488；没有运动或抓取。

- 60秒原生/转换点云883帧，Point-LIO/base均883条，PC原生14.712Hz；最大机身接收间隔80.838ms。
  模式legacy、offset已初始化，两个stamp单调，位姿有限；最大相邻机身位置步长39.951mm。
  这证明该窗口有连续定位输出，不代表绝对定位精度/无漂移已验收。
- 机身TF883/883、相机TF887/887，均100%。目标MAP精确316/316，selected输出980条，管理曾CONFIRMED。
  期间有目标短暂丢失与重获，最后约14秒再次无有效蓝色目标，最终无新鲜CONFIRMED/selected。
- 机身时间实际超前中位679.590ms、P95 762.329ms、范围363.473～765.786ms，远超用户指定150ms。
  READY不会在legacy因新版映射锁存而停止，但不能因此忽略控制器时间新鲜度要求。
- 总结果：`NOT VERIFIED: controller_timestamp_limits, confirmed_selected_target, fresh_confirmed_target`。
  不具备运动条件；结束后入口清理本轮相机/定位，不动已有S1保持。

### 为什么本次不是历史约57ms超前

固定b为3460654.380674488s；采集开始时原始IMU接收-header为3460653.615196966s，
两者相差765.477522ms。转换后首条IMU已超前765.311ms；Point-LIO首条里程计超前765.824ms。
因此偏差已经在转换器产生，不能归为Point-LIO又重复加了一次约64ms扫描时长。
base883/883条完整保留state stamp，也没有重新打时间戳制造超前。

同一段首尾样本的时间增量：

| 时间轴 | 增量秒 |
| --- | ---: |
| PC ROS | 60.000075688 |
| PC wall | 60.000069546 |
| PC CLOCK_MONOTONIC | 60.000069599 |
| PC CLOCK_MONOTONIC_RAW | 59.604846490 |
| GO2 DDS source | 59.619467323 |

PC ROS相对RAW多走395.229ms，GO2源相对RAW多14.621ms。源-PC相对差在本窗口改变约380.608ms，
不能解释为一个稳定的780微秒/秒GO2漂移。固定首帧offset会保留初始化误差，也无法跟随之后的相对变化。
采集在初始化后开始，缺少首帧同时原始采样，因此首帧额外接收延迟和启动期时差变化暂不能分离。

采后只读timedatectl报告NTP offset+86.279ms、网络delay736.170ms、jitter856.961ms；
只读adjtimex(modes=0)返回5/status8257/freq174.054ppm/tick10000us，未修改系统参数。
这些是采后快照，支持检查PC校时/计时环境，但不能证明整个395ms变化都由NTP单独造成。
本次没有改时间算法、再次提高容差、修改系统时间服务或重试刷通过。
先查明并稳定计时基准，再复验legacy；当前不能启动自主停靠。

证据：camera_ws/analysis/go2_legacy_validation_20261113/summary.json、report.json、runtime_excerpt.txt；
原始camera_ws/records/time_20261113_181246_legacy_native_preflight和time_20261113_181306_legacy_localization，
摘要含原始文件SHA256。仅真实采集和记录分析，无离线仿真/批量测试。

## 2026-11-13 历史legacy恢复（实际基线45eac367，原生异常时未启动定位）

本轮先fetch，远端集成分支仍为45eac367f7ca。只从a8f7c667874b提取历史时间转换，
没有回退仓库、Point-LIO算法、D435识别/MAP目标管理或S1协调代码。

### 当前默认与参数

- 默认`time_mapping_mode=legacy`：首帧LiDAR回调计算`b=本机ROS接收ns-原始header ns`，
  本次运行固定；LiDAR和IMU共用`stamp+b`，不改变点内相对时间。
- legacy不创建/调用新版ImuTimeMapping，不受其READY、FAULT或source_guard锁存影响。
  初始化前只等待首帧LiDAR建立共享b，不发布offset=0的另一个IMU时基。
- 日志明确`TIME_MODE=legacy`及`first_cloud_offset_ns`，状态也带mode/offset；
  `ready`在legacy只表示首帧offset已建立，不是新版IMU状态机。
- 保留`imu`和`explicit`可选模式；本轮没有实现滚动中位数或新同步框架。
- **按用户本轮最后的明确要求，future_tolerance已从0.10改为0.15s**，默认值和YAML一致，
  已安装文件也一致；odom_timeout仍0.4s。IMU可选模式内部100ms保护保留，不影响legacy。
- 历史方案曾有约57ms系统性超前，本次恢复不等于精确校时。实际50～100ms超前是否重现，
  必须等原生健康后测量；本轮没有legacy里程计数据，不能把历史值写成本轮值。

### 唯一只读实机入口

先把蓝色目标放到相机视野内，PC执行：

```bash
bash go2_test.sh --time-diagnostic --legacy-check
```

顺序：环境/原生LiDAR+IMU+LidarState检查 → 准备有变化的转换包/控制配置 →
现有伴随主机对时（同终端询问sudo密码，不保存）→ 明确选择legacy →
只读Point-LIO/base/TF/D435/目标MAP/管理器 → 输出原始记录和结果。
不启动速度控制或机械臂动作，不需要手动开多个终端；初始化会建立新地图。
原生检查失败会立即退出，**不会用legacy时间戳掩盖上游故障**。
原生健康后此入口检查连续里程计、400ms新鲜度、当前150ms未来容差、TF、真实MAP及CONFIRMED目标；
20ms诊断目标继续仅记录，不新增验收门槛。默认采30秒；可加`--duration 60`。
通过后如需持续保持只读链可加`--keep-running`；运动前退出该只读会话。

完整运动入口保留不变（当前不得执行，需只读验收通过并确认现场急停/空旷路线）：

```bash
bash go2_test.sh --execute --mode dock
```

该入口默认legacy，保留原生检查、S1 ZERO_HOLD、SEARCH→STOP_CONFIRM→MAP目标追踪→
vx/vy/wz三自由度闭环→4cm/10°/0.5s停稳确认→STOP；不进入抓取。
原full模式的StandDown/StandUp、S1抓取协调与唯一运动出口也保留，不改这些算法或参数。
Ctrl+C请求原停止/清理，物理急停仍由操作者保持可用。
可选IMU模式仍可经`GO2_TIME_MAPPING_MODE=imu`或诊断的`--time-mode imu`选择；
`--legacy-check`始终明确选legacy，避免继承错误模式。未测试Orin硬件，NOT VERIFIED。

### 本轮必要实测与部署结果

- 初次DDS初始化出现接口枚举错误；宿主确认enp12s0为UP/192.168.123.99后，重试取得真实流。
- 10秒81帧原生cloud、2501条IMU、50条LidarState：PC点云 **8.150485Hz**，
  PC IMU **249.987942Hz**；源端cloud_frequency中位 **8.175008Hz**、
  cloud_packet_loss_rate中位 **35.359116%**，cloud_scan_num=180。
  扫描跨度中位108.716ms，P95 148.274ms、最大158.260ms。
  虽不再是2Hz/80%，但仍显著偏离历史约15Hz/低丢包，原生检查未通过。
- 遵照“原生异常直接报告”，**未启动legacy定位链、D435/MAP或运动**。
  Point-LIO持续定位、跳变、里程计实际超前、TF/目标MAP/CONFIRMED均为本轮 **NOT VERIFIED**，
  不借用上一轮数据宣称这次恢复成功，当前不具备单目标停靠放行条件。
- 已通过现有准备入口各构建一次transform_sensors和go2_science_perception；后者仅因future_tolerance
  的代码/配置变化而构建。两个包部署成功，安装文件与源码逐字节一致。
  未编译Point-LIO或其他包；Python/Shell/XML语法和diff检查通过，无离线仿真/批量单元测试。
- 证据：`camera_ws/analysis/go2_legacy_restore_20261113/summary.json`和native_report.json；
  原始数据在主仓库`camera_ws/records/time_20261113_174547_legacy_native_check/`，摘要含SHA256。
  下一步先恢复原生健康，再执行上述同一只读入口；不再研究主控SSH或继续开发时间算法。

## 2026-11-13 蓝色目标与运动前验收（cbf71eb基线，未放行）

本轮真实放置目标（用户回复“完成”），启动D435/Point-LIO/base/目标MAP和目标管理，
没有启动运动控制器、安全门使能或机械臂命令；没有改IMU映射、Point-LIO或MAP/S1控制算法。

### 20ms诊断目标与实际保护

源码：`science_target_approach.py::_fresh`、`science_target_approach.yaml`、`imu_time_mapping.py::map_stamp`。
控制器future_tolerance=0.10s，odom_timeout=0.4s；转换器也保留100ms未来时间保护。
20ms原本是时间诊断期望值，旧入口误将其与硬保护合并为一个阻断检查。
上一轮扫描结束中位20.259855ms，仅比诊断目标大0.259855ms；最大29.536ms仍小于100ms，
不能将其写成控制器100ms保护触发，也不需要改映射算法去跨过0.26ms差异。

本轮只调整验收分类：20ms仍在`diagnostic_goals`报告；硬检查读取现有控制配置的100ms/
400ms，并检查实际里程计相邻接收间隔≤400ms。未放宽转换器或控制器阈值。
因此中位略高于20ms不再独自阻断，但真实丢流、READY失败、TF或目标失败仍阻断。

### 蓝色目标与真实运行结果

- 现场初始伴随主机date又为2022-09-08、uptime约8分钟；不沿用上一轮已对时结论。
  首轮485条相机系目标点、0条MAP点，相机TF 0/894，实测旧时间域解释了转换失败。
- 由用户在本地终端输入密码执行现有sync_go2_clock.py，未把密码写入聊天/文件。
  中间restart入口发生TypeError；改为读取/proc/PID/cmdline的真实参数、记录异常traceback，
  避免把ps渲染文本当作shell参数重新解析。随后启动正确到达时钟检查，测得+172.733ms而阻断。
  再次立即对时后，5次检查+3.836～+3.908ms，通过原50ms检查；没有改对时算法。
- 最终60秒收到816条相机目标点、**440条MAP点/精确转换440/440**、339条selected输出。
  目标管理观察到CONFIRMED未访问目标，实际识别/转换/管理路径已运行成功。
  这只证明当前配置下的转换链，不证明外参标定或目标物理坐标准确度。
- 但是原生点云再次退化：PC **2.065627Hz**、源端中位2.062767Hz、源端中位丢包83.425415%，
  cloud_scan_num仍180、IMU约250Hz；不是靠20ms检查分类能恢复的问题。
- Point-LIO/base只各70条；base超前中位2.129ms、最大80.685ms，机身TF70/70。
  **机身里程计最大间隔737.448ms > 原控制器400ms新鲜度限制**。
- 映射最终锁存`imu mapped time invalid (including scan end); restart localization`，READY=false，
  相机精确TF489/898=54.45%；定位停止后目标转STALE，最终无新鲜CONFIRMED目标。
  MAP转换曾成功440次，但不能把它写成整段持续正常或当前具备运动条件。

**本轮结论：定位不持续，目标链曾跑通但随定位中断失效，不能恢复自主运动。**
不重试运动、不因前半段成功忽略后半段故障，不修改时间映射或放宽任何保护。
先恢复连续原生点云和READY，再用同一入口复验；物理雷达退化触发原因仍未确认。

### 统一入口

放好蓝色目标，PC执行只读验收（不会启动运动）：

```bash
bash go2_test.sh --time-diagnostic --restart-localization --require-target --duration 60
```

脚本记录相机系/MAP点、target_tracks和selected；`--require-target`要求至少5条精确MAP转换、
当前CONFIRMED未访问目标、selected输出及符合原visual_timeout=0.5s/future=0.1s的观测时间。
不使用历史选中过目标替代当前新鲜目标。`--mode dock`在真正运动前自动执行同样的目标验收，
保持原生雷达检查、现有时钟检查、S1 ZERO_HOLD和全部运动保护。

**以下为验收通过后的唯一单目标三自由度停靠命令；当前禁止执行：**

```bash
bash go2_test.sh --execute --mode dock
```

该入口会准备S1保持并重新验收；只有通过后才进入SEARCH/STOP_CONFIRM/三自由度停靠/STOP，
不进入趴下或抓取。现场急停与空旷路线仍需操作者确认；Ctrl+C请求原停止，物理急停保持可用。
本轮只做相关Python语法检查和真实运行，没有离线批量测试/全仓构建或重复构建。
原始记录：主仓库camera_ws/records/time_20261113_170440_blue_target_readiness、
170831_blue_target_synced、171042_target_after_clock、171122_blue_target_final（后三者同time_20261113_前缀）。
摘要/完整报告/运行摘录/SHA256：`camera_ws/analysis/go2_blue_target_20261113/`。
汇总时又遇本机Python异常TypeError，沿既有CPU亲和16-19条件重跑汇总成功；该异常唯一根因未确认。

## 2026-11-13 上游雷达复查：已回到约15Hz，未改原生配置

来源：基线`cc375e6b83f2`、两天原始JSONL/LidarState/元数据、当前真实复测。
本轮不修改时间映射、Point-LIO、MAP接近、三自由度停靠或S1程序，不运动/抓取。

| 真实窗口 | PC接收Hz | 源端自报Hz中位 | 源端丢包%中位 | 扫描跨度ms中位 | cloud_scan_num |
| --- | ---: | ---: | ---: | ---: | ---: |
| 11-12 23:50 正常 | 14.843 | 14.862 | 1.105 | 63.760 | 180 |
| 11-13 16:33 异常 | 1.225 | 1.213 | 86.188 | 717.437 | 180 |
| 11-13 16:50 最终复测 | 14.771 | 14.838 | 1.105 | 63.771 | 180 |

**180没有发生变化，也没有证据表明它在这台GO2上异常，不能照搬独立SDK默认18覆盖。**
三种状态的消息布局相同，实际单帧点数中位1405/1416/约1427，非点数增加十倍；
固件1.0.2、软件1.0.0.38、SDK1.0.9相同，PC均为Domain0/CycloneDDS/enp12s0。
源端原生发布配置文件未能读取；不能把这些状态字段对照称为完整配置diff。
11-12与11-13的生产者GID不同，但11-13异常与当前恢复的GID相同。
用户确认异常采集后未重启、未插接、未切App模式；本轮没有发送配置/重启命令。
当前只能记录**无已知操作下自行恢复**，具体触发原因Unknown，不能宣称修复了某个配置。

故障定位：异常时源端LidarState与PC接收频率都约1.2Hz，并且源端已编码进单条点云的
点内时间跨度变大。仅PC丢掉整帧不会改变幸存消息内部扫描跨度，所以故障至少在原生
点云组装/生产阶段可见。源端高丢包与收集相近点数需更长时间相符；究竟是串口链路、
解析/调度或其他生产者问题，缺少源端日志不能进一步归因。PC额外丢帧不能被完全排除。

### 官方接口核对（只读查阅，未执行写入）

- [L1 SDK正式头文件](https://github.com/unitreerobotics/unilidar_sdk/blob/main/unitree_lidar_sdk/include/unitree_lidar_sdk.h)：
  `initialize(cloud_scan_num=18, port, ...)`和`initializeUDP(...)`将同名参数传给直连reader；
  `printConfig()`可打印该reader配置，`reset()`、`setLidarWorkingMode(NORMAL/STANDBY)`是直连接口。
  这些不是GO2原生DDS的组帧恢复RPC。未取得原生生产者设备/配置访问，不新建reader抢占雷达。
- [L1 ROS2 wrapper](https://github.com/unitreerobotics/unilidar_sdk/blob/main/unitree_lidar_ros2/src/unitree_lidar_ros2/include/unitree_lidar_ros2.h)：
  读取cloud_scan_num ROS参数后在初始化时传入SDK，runParse返回POINTCLOUD才发布。
  这能确认独立驱动的配置入口；其与GO2裸DDS生产者不是同一已验证实现。
- [官方GO2 LidarState消息](https://github.com/unitreerobotics/unitree_ros2/blob/master/cyclonedds_ws/src/unitree/unitree_go/msg/LidarState.msg)
  只声明cloud_scan_num字段，没有定义它与独立SDK组帧参数的对应关系。
  因此GO2的180是否严格表示每条DDS点云组帧数量 **Unknown / needs verification**，不按字段同名推定。
- [RobotStateClient](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/robot_state/robot_state_client.hpp)
  只公开服务列表/切换/状态上报频率，不提供已文档化的原生点云组帧参数设置。
  本次已查官方SDK未找到`/utlidar/client_cmd`可核验的配置恢复payload；不向其猜测发命令。
- 官方文档中心L1/GO2 LiDAR页面正文不可读取（直读遭EdgeOne拦截），未把搜索摘要或第三方
  用户转述当成已核验的GO2命令。独立SDK方法和GO2内部驱动恢复方法必须分开。

### 一键入口与实机结果

原生检查已加入现有入口，独立调用如下（只订阅，无SSH/构建/启停/配置写入）：

```bash
bash go2_test.sh --time-diagnostic --lidar-check --duration 10
```

输出PC接收频率、源端频率/丢包/scan_num、扫描跨度及上游/PC路径判断。
粗健康线为PC及源端中位频率≥10Hz、源端中位丢包≤10%、扫描中位≤100ms，
来自约15Hz/64ms正常参考，仅用作原生输入检查，不是修改转换器/控制器保护。
缺LidarState或流则明确失败。`PASS: native LiDAR feed only`只代表原生输入，不代表时间或运动通过。
完整`--execute`在对时、S1使能和定位前自动跑8秒同一检查；定位后的完整时间/TF检查继续保留。
纯`--observe-only`仍保留原生时钟域报告，不用源端旧epoch否定点云频率恢复。

最终30秒：443帧原生点云、7491条原生IMU、443条Point-LIO及443条base里程计；
原生丢包最大2.762%、扫描跨度P95 69.275ms、最大113.828ms。映射ready且无fault，
offset为+3454302.709204369s；base时间超前中位16.394ms、P95 24.654ms、最大27.836ms。
机身精确源时间MAP TF **443/443**，相机 **447/447**，均100%。
因此**最终观测窗口连续里程计已恢复**，不是对长期稳定性或定位精度的验收。

完整报告仍为`NOT VERIFIED: old_systematic_future_removed`：扫描结束时间中位
**20.259855ms**，略大于原20ms诊断线。未修改该门限，也未修改100ms控制器future保护。
此前本轮第一次只读复测仅98条里程计后触发IMU时间保护，失败证据一并保留，不用最后窗口抹除。
D435确有图像，但无有效蓝色目标，target_map_debug=0，实际目标MAP转换仍NOT VERIFIED。
所以**本轮不放行自主停靠**；先按原诊断通过并看到有效目标MAP后，才由操作者执行原dock入口。
不为跨过门槛继续优化时间算法或重复刷通过。

证据：`camera_ws/analysis/go2_native_lidar_20261113/comparison.json`、两次定位report及运行摘录；
原始数据在主仓库`camera_ws/records/time_20261113_164438_upstream_current/`、
`time_20261113_164555_restored_lidar_mapping/`、`time_20261113_164936_native_entry_verified/`、
`time_20261113_165049_upstream_full_recheck/`，摘要含SHA256与真实生产者GID。
仅做相关脚本语法检查和真实运行采集，无离线测试、无全仓构建。本轮失败后清理拥有的只读链。

## 2026-11-13 实机复测：Point-LIO未恢复，禁止恢复运动

来源：基线 `b5ca59876d99`，本轮真实ROS/DDS采集和进程日志；没有发送运动命令。
通过现有入口实际构建transform_sensors及go2_science_perception，启动Point-LIO、转换器、
base_odom、D435和目标MAP链。开始时无旧定位/控制任务，重启入口记录的替换PID列表为空。
失败后入口按既有生命周期清理节点，因此采集结束后查不到进程不表示未启动。

- 最终窗口收到38帧原生点云、7686条IMU、154条LidarState、458帧D435图像。
- 雷达自报cloud_frequency中位1.212644Hz，cloud_packet_loss_rate中位86.187843%；
  IMU中位249.694962Hz、丢包0、error_state=0。PC网卡RX errors/dropped均0。
  点内扫描跨度中位717.437ms、P95 1168.972ms、最大1336.256ms；首轮最大1424.244ms。
- 最终冻结offset为 **+3454302.679386230s**（首轮+3454302.634609028s），只是IMU近似映射。
  转换器锁存`cloud mapped time invalid (including scan end); restart localization`；
  状态保留的扫描结束偏差为-63.785ms，扫描起点年龄保护阻断，不能靠增加年龄上限恢复。
  首轮残差-0.027ms、最终残差24.886ms均是故障锁存前窗口的冻结统计，不是持续稳定性证明。
- 正式30秒窗口转换cloud/IMU、state/base里程计均0条：**实际里程计时间误差不可测，不能报0ms**。
  相机精确源时间map TF **0/450（0%）**，base无查询；目标MAP debug 0条。
  camera_target_to_map有实际目标点，但定位TF停在旧时刻，出现TF_LOOKUP_DEFERRED/
  pending_timeout，最后观测相差36.179s；D435目标MAP转换未通过。
- 伴随主机启动日期2022-09-08。由用户在本地终端输入密码后执行现有对时脚本；
  曾立即测到+4.108～+4.211ms，但后续检查+136.563ms及-109.036ms仍超过50ms。
  PC NTP处于调频（观测-477.034ppm）；时差变动的唯一原因尚未确认，不只归因于调频。
  最终使用已启动相机的existing只读模式采集payload，**不代表主机对时验收通过**。
- 用户确认雷达无遮挡；旧记录20261112_235022首个LidarState为14.728Hz/1.657%丢包，
  本次退化确实在原生输入可见。硬件/上游生产者丢包根因仍Unknown，不凭error_state=0排除故障。
  /utlidar/server_log单次8秒观察没有收到日志，不冒称设备无异常。

本轮最小修复仅两处诊断入口配置：沿用当前感知配置的真实图像话题
`/camera/camera/color/image_raw`（同时用于已有流检查），默认DDS补上现有伴随主机peer。
修复后458帧图像真实到达；没有改IMU映射算法、Point-LIO滤波器、阈值、MAP/三自由度停靠/S1算法。
只执行必要包构建、脚本语法及真实采集；没有增加离线测试。
证据：`camera_ws/analysis/go2_hardware_20261113/summary.json`及各轮report，
原始JSONL/启动日志保留在主仓库`camera_ws/records/time_20261113_*`，摘要含SHA256。

当前处置：先恢复原生雷达连续点云并稳定PC/伴随主机时钟，再运行原只读入口重新验收；
仅重启PC定位无法修复当前上游输入。未取得已确认的源端恢复命令，不发送猜测配置、不重启整机。
本轮不启动控制器/安全门或S1保持；运动运行条件尚不具备，完整运动/抓取仍 **NOT VERIFIED**。

只读复验入口（不运动）：

```bash
bash go2_test.sh --time-diagnostic --restart-localization --keep-running
```

**以下是验收通过、退出只读链、现场确认急停/空旷环境后的唯一下一条运动测试命令；当前禁止执行：**

```bash
bash go2_test.sh --execute --mode dock
```

该入口会重新对时（可能要求本地sudo密码）、确认S1 ZERO_HOLD、连续传感器/里程计和TF、
运行同一时间验收，再进入SEARCH→STOP_CONFIRM→三自由度单目标停靠→STOP，不进入趴下/抓取。
运动前仍须通过原0.10s future保护、0.4s odom freshness等；没有自动避障。
Ctrl+C请求原停止/清理，物理遥控急停需由操作者保持可用。本轮未验证物理停靠结果。

## 2026-11-13：IMU辅助统一时间映射（代码完成，实机 NOT VERIFIED）

本轮基于已fetch核对的 `bac3a992feb7` 完成代码；提交号由本节所在Git提交标识，运行报告自动记录实际HEAD。
按用户“先完成代码部分”，本轮未构建overlay、未部署、未重启现有SLAM、未运行硬件采集或运动。
历史证据：原生DDS源时间/IMU落后PC约3390496秒；旧base_future中位57.215ms、P95 67.361ms。
这些是上一轮记录，**本轮实际offset、修复后里程计偏差、定位恢复、D435目标MAP转换均 NOT VERIFIED**。

统一入口仍为：

```bash
bash camera_ws/scripts/run_go2_time_diagnostic.sh
```

运行时仅在源码摘要变化时构建 `transform_sensors`，缺少感知overlay时单包准备；
已有定位默认复用，不自动重置地图。没有现有定位时启动原有只读SLAM/D435/TF/目标MAP链，
初始化后采集30秒，输出原始JSONL、统计/趋势、offset残差、扫描结束时间、里程计、精确TF和前后对比。
不会启动运动控制器或S1；发现运动控制节点/速度发布者则拒绝修改定位会话。
默认退出只清理本入口启动的节点，原有会话保留。报告通过仅代表观测窗口指标，不代表运动/抓取通过。

当前机器若仍运行旧阻断转换器，实际验证时用同一入口的显式新地图模式：

```bash
bash camera_ws/scripts/run_go2_time_diagnostic.sh --restart-localization --keep-running
```

该选项会结束已识别的旧只读定位/感知进程、创建新地图；保留原日志。报告通过后保持只读链，Ctrl+C退出。
原生DDS不干预模式改为同入口 `--observe-only`；`--reuse-only` 仅验证既有链；`--prepare-only` 只准备overlay。
`bash go2_test.sh --time-diagnostic` 等价分发；`bash go2_test.sh --execute` 为真实运动，
接入新overlay，并在只读链就绪后完成同一诊断，未通过不进入SEARCH/停靠/抓取；本轮没有执行。

算法：`t_local=t_sensor+b`，a=1；2秒窗口至少200条原始IMU，取 `receive_ROS-header` 的P5，
P95-P5须不超过20ms，DDS source-header用于核验IMU发布延迟（允许-1～20ms），不当成独立UTC。
b初始化后冻结，LiDAR/IMU共用，不修改点内相对时间、不按帧用now覆盖、不减固定64ms。
初始化前两条转换流均不发布；重复/倒退样本丢弃；明显时钟跳变、映射年龄异常或
映射扫描结束超前100ms锁存停止输出，需新会话恢复。状态 `/utlidar/time_mapping/status` 每秒发布。
这是包含未知单向网络/调度延迟的近似映射，**没有修好源主控系统时钟**；不自动估计时钟倍率。
长期漂移由offset候选残差报告，是否需要a修正待真实长时数据；瞬时突变禁止在线重锁。

通过条件包括连续原始/转换流与里程计、无倒退、有限位姿、use_sim_time=false、无/clock、
扫描结束及两路里程计未来偏差中位≤20ms且最大≤100ms、按原始观测时间的机身/相机TF成功率≥95%。
该20ms是诊断验收门限，**没有修改控制器future_tolerance=0.10s**。
目标MAP debug另行统计；无可见目标时明确NOT VERIFIED，不把图像TF成功写成目标转换已通过。
位姿相邻步长只记录，不凭它独自宣称无定位跳变。

核心映射无PC目录/IP依赖，以运行主机ROS时间映射；部署Orin时可设置
`GO2_TIME_UNDERLAY`、`GO2_TIME_INTERFACE`、`GO2_CAMERA_HOST=local`。
PC默认仅复用已有相机流；若相机无发布者，使用原有 `.18` 相机伴随主机密钥和已部署相机脚本，
先只读检查伴随主机时钟，不设置任何主机时钟。相机选项 `--camera-host existing` 可完全禁止SSH。
`GO2_CAMERA_SSH_KEY` / `GO2_CAMERA_LAUNCH` 可覆盖原路径；不访问 `.161` 源主控SSH。
伴随主机时差超标、相机脚本/密钥缺失会明确失败；不会借此修改雷达配置或套用L2同步API。

修改范围：转换器及小型IMU映射模块、launch时间模式参数、单入口采集/生命周期、完整测试接入、文档。
Point-LIO C++定位算法/滤波器、MAP速度/控制参数、HSV/D435识别、目标管理、机械臂动作链均未修改。
静态验证：5项时间映射边界检查、相关Python语法、shell语法、XML解析和git diff检查；不是构建或实机证据。


## 历史入口行为：纯PC DDS + LidarState（bac3a99，现需 --observe-only）

```bash
bash camera_ws/scripts/run_go2_time_diagnostic.sh
```

本版不需要登录任何GO2主机：Domain 0、enp12s0，仅订阅真实DDS消息。
先发现LidarState实际名称和类型，再连续采集30秒；没有该Topic就记录不存在，不伪造。
不SSH、不构建、不启动/停止SLAM、不调用运动/雷达配置或同步接口。
输出原始cloud/IMU、DDS source timestamp、LidarState全部字段和PC ROS/wall/monotonic时间，
以及各时间差的统计/趋势。现有SLAM保持不动，末尾返回码2表示时钟域仍不一致。
当前实测：`/utlidar/lidar_state`，`unitree_go/msg/LidarState`，149条/30秒，能够读取。
用户已确认型号L1；实读固件1.0.2、软件1.0.0.38、SDK1.0.9。
DDS源时间、状态stamp和serial_recv_stamp都落后PC约3390496.05秒，
所以问题已经存在于发布端时间域；不能据此声称已直接读到主控OS日期。

L1公开接口的getTimeDelay只读取链路延迟，TIMESYNC是解析到的消息类别，
没有查到可通过当前DDS连接设置源主控系统时间的公开方法。
L2的syncLidarTimeStamp/use_system_timestamp不可套用到这台L1。
当前没有执行任何同步操作；需要宇树确认该固件支持的无需SSH主控校时通路
（主控侧官方NTP/SNTP配置或官方维护/固件时间服务），不能伪造RobotStateClient时间API。
LidarState本身是观测接口，不能把serial_recv_stamp或PC接收时间当成独立UTC校准基准。

本轮数据：`camera_ws/records/time_20261112_235022_pc_dds_lidar_state/`。
实测摘要与旧数据重算：`camera_ws/analysis/go2_time_sync_20261112/pc_dds_lidar_state_summary.json`。
本节仅为bac3a99历史记录；当前行为以上面的2026-11-13节为准。

## 旧版时间诊断记录（12d0db5，非当前默认行为）

当前原生传感器时间域与PC相差约3,390,496秒，尚无独立校准基准。
已移除“首个完整点云接收时间减header”自动偏移；现在源时间不可信会明确阻断输出。
**源时钟尚未同步，修正后有效里程计及完整运动任务仍NOT VERIFIED；不要把0条里程计当作0误差。**

在当前仓库目录只需：

```bash
bash camera_ws/scripts/run_go2_time_diagnostic.sh
```

自动检查连接、主机/NTP状态、现有节点与TF，必要时只构建有变化的两个包，
已有SLAM时只订阅，不重启；没有SLAM时才启动只读链。连续采集30秒，统计各阶段
均值/中位数/P95/极值/趋势，保存原始JSONL、内部beg/end/current日志和前后对比。
不启用机械臂、不起立/趴下、不发布任何速度。报告位于主仓库
`camera_ws/records/time_日期_时间_标签/`；`report.txt`、`report.json`、`trend.json`可直接查看。
缺少源时钟基准时返回码2，报告`BLOCKED_NATIVE_CLOCK`，这是明确阻断，不是成功验收。

本机主目录的同名脚本转发到独立集成目录`/tmp/go2_onekey_s1`；目录丢失时自动恢复
实验分支并链接现有导航底包/记录目录，不切换主仓库分支、不覆盖脏源码。
集成分支也支持根入口`bash go2_test.sh --time-diagnostic`，共用同一实现。
完整`--execute`入口先跑同一只读诊断；未通过会在机械臂使能/运动之前停止。
原生已同步时保留原header；只有经独立时钟测量的映射才可设置
`sensor_time_offset_sec`，默认0，禁止把约39天差值或接收延迟手填成“标定”。

当前阻塞：`.18`是可登录的相机/S1伴随主机，NTP未启用；原生源时间域并不随其对时
自动校准。主控候选`.161`现有账户无法登录，标准NTP查询也无响应，尚不能确认/修复
源主机系统时间。需取得源主控权限或可验证的双向时间同步接口后继续；不扩100ms容差。
实测基线机身里程计未来偏差：中位57.215ms、P95 67.361ms、最大71.510ms。
修正后30秒采到445帧原始点云、7495条原始IMU，未校准时转换/里程计输出均0条。
详细源码与测量见`camera_ws/docs/experiments.md`末节及
`camera_ws/analysis/go2_time_sync_20261112/measured_summary.json`。

本节及对应提交为当前实验状态；下文旧运动流程只在时间诊断通过后使用。

## 一句话运行最新全流程（2026-11-12）

在PC终端执行（从任意目录均可）：

```bash
bash /home/czx666/today_full_20260901/today_full/go2_test.sh --execute
```

**会真实回零使能机械臂、移动GO2、趴下、抓取和起立。** 先完成接线/上电，
核对S1初始位与支撑，路线空旷，遥控急停在手，目标恢复到起点前方；无自动避障。
本命令是操作者对本轮动作的明确启动；不再分终端手动复制所有命令。
对时和停旧自定义遥控服务仍可能询问GO2 sudo密码，密码不保存。需能访问Git远端。

自动顺序：独立克隆/更新 `feature/map-approach-s1-grasp-integration` → 单包构建 →
有线IP/设备/旧进程检查 → 对时与偏差检查 → 同步适配器 → 停旧自定义路由服务 →
复用健康S1或启动回零保持 → D435 → PC定位感知TF → 连续新鲜数据预检 →
SEARCH → STOP_CONFIRM → 三自由度停靠 → STOP → StandDown → S1抓取 → ZERO_HOLD → StandUp。
复用原协调器及原保护，不修改控制算法。停靠失败不会进入抓取。

- 不加 `--execute`：只打印离线计划，不连接设备、不构建。
- 只做停靠：`bash go2_test.sh --execute --mode dock`，仍会启动S1保持及GO2行走。
- 代码目录改为 `/tmp/go2_onekey_s1`，构建在其 `.onekey/`；原主分支与脏改动保留。
  每轮读取远端最新提交并记录hash；本地额外提交/脏改动拒绝覆盖，网络失败不回退旧版。
  相同commit且本入口有成功构建标记时复用overlay；新commit重新构建。
  自建的两个链接必须指向预期主仓库路径；成功构建标记位于独立clone的.git目录。
- 检测到旧相机/定位/控制或忙碌/故障S1时停止，按后文人工处理，不自动杀旧任务。
- `Ctrl+C`请求结束本轮任务并收尾；真实停止仍需现场确认，遥控急停始终可用。
  无自动失败重试或故障起立；任务拒绝退出时保留定位，提示物理急停并等待退出。
- 退出关闭本轮相机/定位，**S1保持服务继续运行**，防止行走或故障时失去机械臂保持。
  断电/关服务前先按第10节确认已停止、任务结束和机械臂有支撑。
- 总控日志：主仓库 `camera_ws/records/full_时间戳/`；含commit、camera/SLAM日志。
  控制、录像、最终run JSON仍在 `camera_ws/records/`；S1保持日志路径启动时打印。

2026-11-12已实际从连接进入SEARCH、STOP_CONFIRM和三自由度停靠；停靠中机身里程计
时间戳超前108.354ms，触发原100ms保护，未趴下/抓取。用户确认已停止。
9项离线测试通过，原始/叠加录像均完整605帧；启动器已修正链接、对时RTT、后台服务和
转换后传感器预检问题。**完整停靠与抓取仍未通过，时间戳底层原因未解决**。
直接重跑不保证消除该故障；本命令用于操作者准备并授权的新一轮测试。
下文旧命令的 `/tmp/go2_map_s1_integration_20261111` 与旧overlay路径
属于手动模式，不与一键入口同时启动。

**当前结论：旧 MAP 搜索接近已有实测成功记录；S1 抓取停靠实验版仍未通过末端停靠，不能当作已验证的一键抓取版本。**
历史实测包含集成分支 `865946a` 的vx/wz版本。当前vx/vy/wz版本已实际进入停靠，
但因上述时间戳保护中断，**尚未通过停靠验收**。

## 0．先记住目录和终端

| 名称 | 本机实际值 |
|---|---|
| PC 主仓库（本文件所在最外层） | `/home/czx666/today_full_20260901/today_full` |
| S1 集成代码工作目录 | `/tmp/go2_map_s1_integration_20261111` |
| S1 集成分支 | `feature/map-approach-s1-grasp-integration` |
| 稳定 MAP 分支 | `feature/d435-perception-workspace`（已知基线 `e15c66c`） |
| 不用于本文的视觉伺服分支 | `feature/d435-visual-servo-optional` |
| PC 有线接口 / IP | `enp12s0` / `192.168.123.99` |
| GO2 SSH | `unitree@192.168.123.18` |
| SSH 私钥 | `/home/czx666/.ssh/id_ed25519_go2` |
| GO2 S1 交付包目录 | `/home/unitree/s1-arm-control` |
| GO2 S1 Python | `/home/unitree/s1-arm-py310/bin/python` |
| GO2 S1 串口 | `/dev/ttyUSB0` |
| GO2 S1 服务 socket | `/tmp/s1-arm-integration.sock` |
| PC 本轮构建 overlay | `/tmp/science_s1_install/setup.bash` |

建议开五个终端：A=PC准备/构建，B=GO2机械臂服务，C=GO2相机，D=PC定位感知，E=PC测试/日志。
文中“PC执行”是在PC本地终端；“GO2执行”是先SSH登录后的终端。后台启动过的节点不能再启动第二份。
`/tmp` 可能被清理；目录丢失时先看第12节恢复方法，不要在主仓库强制切分支或清掉已有改动。

## 1．接线、上电、检查网线（PC终端A）

1. GO2与PC接网线。D435接GO2；S1的USB串口和腕部D455也接GO2。
2. S1按交付要求放在初始零位附近，检查线束。路线空旷，只放一个蓝色目标，遥控急停在手。
3. 若要进行完整任务，目标放在当前朝向前方约2.5～3m附近；不要从上轮目标已在右后方的位置直接重新搜索。

```bash
ip -br addr show enp12s0
ip route get 192.168.123.18
ping -c 3 192.168.123.18
ssh -i /home/czx666/.ssh/id_ed25519_go2 unitree@192.168.123.18
```

SSH成功后，在GO2终端查看设备：

```bash
lsusb
ls -l /dev/serial/by-id/ /dev/ttyUSB* /dev/ttyACM*
/home/unitree/s1-arm-py310/bin/python -c 'import pyrealsense2 as r; print([(d.get_info(r.camera_info.name), d.get_info(r.camera_info.serial_number)) for d in r.context().query_devices()])'
```

2026-11-12已枚举：D435=`213622070598`，D455=`231622302209`；后者与S1交付包标定记录一致。
若重新接线后序列号不同，应核实实际相机；不要改标定JSON来冒充原相机。
没有`/dev/ttyUSB0`时，先处理接线/供电/串口枚举，不用板载`ttyTHS*`替代。

## 2．对时：先对时，再开相机和SLAM（PC终端A）

若上次测试进程还在，先按第10节停止运动并关闭旧相机/SLAM，再改时间。
GO2重启后可能回到2022年；PC本身的日期也要正确。

```bash
date -Is
timedatectl status
cd /tmp/go2_map_s1_integration_20261111
/usr/bin/python3 camera_ws/scripts/sync_go2_clock.py
ssh -i /home/czx666/.ssh/id_ed25519_go2 unitree@192.168.123.18 'date -Is'
```

脚本会在本地终端询问 **GO2 sudo密码**，输入时不回显，不写进命令、文件或聊天。
如果时间刚改过，旧相机和定位链必须重新启动；不要沿用改时前的时间戳。

## 3．准备阶段关闭旧机械臂入口和旧S1服务（GO2终端B）

**先确认GO2已停住、当前没有抓取任务，机械臂有稳定支撑，再关服务。关闭服务会解除软件零位保持。**
GO2行走时必须保留S1零位保持，不能先关机械臂再让狗继续走。

先关闭旧的自定义遥控机械臂入口，防止争抢串口；这不是关闭GO2原厂遥控器：

```bash
sudo systemctl stop go2-remote-router.service
systemctl is-active go2-remote-router.service
```

检查新服务是否在运行：

```bash
pgrep -af '[r]un_arm_integration.py'
/usr/bin/python3 /home/unitree/s1-arm-control/scripts/arm_task_client.py status
```

如果是前台启动的服务，在它的原终端按`Ctrl+C`。
如果是本轮旧的nohup方式启动，下面命令只匹配该S1入口：

```bash
pkill -TERM -f '^/home/unitree/s1-arm-py310/bin/python -uB scripts/run_arm_integration.py( |$)'
pgrep -af '[r]un_arm_integration.py'
fuser /dev/ttyUSB0
```

服务收到SIGTERM会先等正在进行的抓取完成再退出；不要把它当成机械臂急停。
若`status`显示GRASPING/RETURNING_ZERO/FAULT_*，先处理当前任务/故障，不继续发新的任务。
`pgrep`无输出、`fuser`无占用，才表示旧进程退出。socket文件存在本身不表示服务健康。

**仅当你曾安装systemd常驻服务时**，用下面方法停止；普通前台启动不用运行它：

```bash
sudo systemctl stop s1-arm-integration.service
```

## 4．核对代码、构建本轮包（PC终端A）

不要在主仓库覆盖未提交改动。集成测试使用独立工作目录：

```bash
cd /tmp/go2_map_s1_integration_20261111
git status --short
git branch --show-current
git fetch origin
git log -1 --oneline
git log -1 --oneline origin/feature/map-approach-s1-grasp-integration
```

确认分支是`feature/map-approach-s1-grasp-integration`。需要更新且本地没有未提交改动时才执行：

```bash
git pull --ff-only origin feature/map-approach-s1-grasp-integration
```

已有导航底包位于主仓库的`deploy_ws/install`。首次恢复新工作目录时补这两个链接；已存在则跳过：

```bash
cd /tmp/go2_map_s1_integration_20261111
if [ ! -e deploy_ws ]; then
  ln -s /home/czx666/today_full_20260901/today_full/deploy_ws deploy_ws
fi
mkdir -p /home/czx666/today_full_20260901/today_full/camera_ws/records
if [ ! -e camera_ws/records ]; then
  ln -s /home/czx666/today_full_20260901/today_full/camera_ws/records camera_ws/records
fi
```

只构建本轮感知/接近包，不全仓库构建：

```bash
cd /tmp/go2_map_s1_integration_20261111
source /opt/ros/humble/setup.bash
source deploy_ws/install/setup.bash
colcon --log-base /tmp/science_s1_log build --base-paths camera_ws/src \
  --build-base /tmp/science_s1_build \
  --install-base /tmp/science_s1_install \
  --packages-select go2_science_perception
export SCIENCE_APPROACH_OVERLAY=/tmp/science_s1_install/setup.bash
source "$SCIENCE_APPROACH_OVERLAY"
```

底包`deploy_ws/install/setup.bash`若不存在，当前机器缺少原部署环境；上述单包构建不能代替导航底包安装。
没有改代码且overlay仍在时，不必每轮重新构建。改过包内Python/参数后必须用新构建结果。

同步GO2任务适配器（更新PC代码后执行，不会自行触发动作）：

```bash
ssh -i /home/czx666/.ssh/id_ed25519_go2 unitree@192.168.123.18 'mkdir -p /home/unitree/go2_map_s1'
scp -i /home/czx666/.ssh/id_ed25519_go2 \
  camera_ws/scripts/launch_go2_d435_foxy.sh \
  camera_ws/scripts/s1_task_host.py \
  camera_ws/src/go2_science_perception/go2_science_perception/s1_task_sequence.py \
  unitree@192.168.123.18:/home/unitree/go2_map_s1/
```

## 5．启动S1常驻零位保持（GO2终端B）

不要另开第二个串口程序。保持这个终端不关闭。
先离线预检，不开串口或相机：

```bash
cd /home/unitree/s1-arm-control
OPENBLAS_NUM_THREADS=1 /home/unitree/s1-arm-py310/bin/python -B scripts/run_arm_integration.py
```

应输出`RESULT = OFFLINE_PREFLIGHT_OK`。随后真实使能机械臂并回零保持：

```bash
OPENBLAS_NUM_THREADS=1 /home/unitree/s1-arm-py310/bin/python -uB \
  scripts/run_arm_integration.py --dev /dev/ttyUSB0 --execute
```

等待`INTEGRATION_READY = ZERO_HOLD`。在另一GO2终端查状态：

```bash
/usr/bin/python3 /home/unitree/s1-arm-control/scripts/arm_task_client.py status
```

行走前应为`state=ZERO_HOLD`、`busy=false`、`navigation_allowed=true`。
若提示`initial pose outside existing zero-start windows`，按原机械臂方法恢复初始位置；不改启动窗口。
本轮曾因J5=-0.126841rad超出±0.08rad被拒，恢复初始位置后成功启动。
若是FAULT_HOLD/FAULT_IO，不起立、不继续导航、不反复发送grasp。

## 6．启动D435（GO2终端C）

先查看是否已启动；已有一份就不要重复运行：

```bash
pgrep -af '[r]ealsense2_camera_node'
```

没有相机进程时启动，保持终端不关闭：

```bash
GO2_D435_SERIAL=213622070598 bash /home/unitree/go2_map_s1/launch_go2_d435_foxy.sh
```

这里明确绑定D435，腕部D455留给S1。不要额外给D455启动ROS相机驱动与抓取模块争抢。

## 7．启动PC定位、目标感知与TF（PC终端D）

```bash
cd /tmp/go2_map_s1_integration_20261111
source /opt/ros/humble/setup.bash
source deploy_ws/install/setup.bash
source /tmp/science_s1_install/setup.bash
source camera_ws/scripts/science_approach_pc_env.bash
ros2 launch go2_science_perception d435_slam_target_validation.launch.py \
  remote_camera:=true use_rviz:=true
```

不需要RViz时将`use_rviz:=true`改成`false`。
这一步启动Point-LIO、粗定位、camera_target_to_map、target_manager与TF，不启动行走。
必须使用`remote_camera:=true`，避免PC再起一份相机驱动。
等待雷达初始化完成；保持本终端运行。

## 8．打开状态终端，选择一种测试（PC终端E）

新PC终端需要ROS环境时，先执行：

```bash
cd /tmp/go2_map_s1_integration_20261111
source /opt/ros/humble/setup.bash
source deploy_ws/install/setup.bash
source /tmp/science_s1_install/setup.bash
source camera_ws/scripts/science_approach_pc_env.bash
export SCIENCE_APPROACH_OVERLAY=/tmp/science_s1_install/setup.bash
```

另外开一个PC终端看实时状态（只读）：

```bash
cd /tmp/go2_map_s1_integration_20261111
bash camera_ws/scripts/watch_approach_logs.sh
```

以下测试每次只选一项，不能同时启动；都有SEARCH最大5m、0.25m/s、SEARCH总60s、STOP_CONFIRM总6s。
合成接近速度上限0.30m/s。普通MAP接近仍限25s；抓取停靠取消固定总时限，保留10s无进展停机及独立ODOM 5m/25s保护。没有自动避障，路线必须由人保证空旷。

### A．普通MAP搜索接近，不做S1右侧停靠、不趴下抓取【真实运动】

```bash
cd /tmp/go2_map_s1_integration_20261111
export SCIENCE_APPROACH_OVERLAY=/tmp/science_s1_install/setup.bash
bash camera_ws/scripts/run_map_anchor_experiment.sh \
  --execute --search --search-distance 5 --search-timeout 60 --timeout 25
```

这里复用集成分支中保留的普通MAP模式，沿用原相机XY 0.10m停车条件，不启用grasp-pose。
它与S1右侧抓取停车位置不同。若需要严格使用原稳定分支源码，见第11节。
S1安装在狗上时仍应保持ZERO_HOLD；这个旧入口本身没有新增S1服务监控，启动前手动确认状态。

### B．仅测试S1抓取停靠，不趴下、不抓取【真实运动】

```bash
cd /tmp/go2_map_s1_integration_20261111
export SCIENCE_APPROACH_OVERLAY=/tmp/science_s1_install/setup.bash
bash camera_ws/scripts/run_map_s1_task.sh \
  --execute --search-distance 5 --search-timeout 60
```

流程：SEARCH → STOP_CONFIRM → APPROACH_TO_GRASP_POSE → STOP。
位置≤4cm、航向≤10°且静止持续0.5s才完成；S1预测矩形仅记录参考。新控制同时允许前后、左右和转向，不能只看“离物块很近”。
停车后实际测量目标相对S1基座XY：+X是狗后方，+Y是狗右侧；与日志预测分别记录。

### C．完整单目标流程【真实运动＋趴下＋机械臂抓取＋起立】

```bash
cd /tmp/go2_map_s1_integration_20261111
export SCIENCE_APPROACH_OVERLAY=/tmp/science_s1_install/setup.bash
bash camera_ws/scripts/run_map_s1_task.sh --execute --full \
  --search-distance 5 --search-timeout 60 \
  --posture-profile camera_ws/src/go2_science_perception/config/science_s1_posture_go2_20261112.json
```

流程：SEARCH → STOP_CONFIRM → 抓取停靠 → STOP → StandDown → S1原完整抓取 → ZERO_HOLD → StandUp。
无需再手填四个参数。本机已测：趴下反馈高度上限0.084m、起立下限0.304m，两种mode均0。
这些是本机反馈判据，不是S1基座高度。更改机械安装/支撑条件后不能直接视为仍有效。
停靠失败时不会趴下或抓取。`accepted=true`只代表接收任务，必须等待任务完成及ZERO_HOLD。
2026-11-12最后一次停在机身里程计未来时间戳保护，未实际触发抓取，完整一键任务仍未通过。

### D．仅打印完整计划【不打开硬件、不运动】

```bash
cd /tmp/go2_map_s1_integration_20261111
/usr/bin/python3 camera_ws/scripts/run_map_s1_task.py --full \
  --posture-profile camera_ws/src/go2_science_perception/config/science_s1_posture_go2_20261112.json
```

## 9．最少量只读检查和日志查看（按需执行）

先加载第8节PC环境。下面`hz`命令持续输出，查看后按`Ctrl+C`结束；不要一次全部长期运行。

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ros2 topic hz /utlidar/cloud
ros2 topic hz /utlidar/imu
ros2 topic hz /state_estimation
ros2 topic info /cmd_vel_raw --verbose
ros2 run tf2_ros tf2_echo map vehicle
```

相机内参单帧：

```bash
ros2 topic echo --once --qos-reliability best_effort /camera/camera/color/camera_info
```

测试程序会自动记录状态JSON、控制/安全门日志、原始和叠加画面；本轮录像正常关闭后才退出。
数据实际保存在主仓库，不应只找/tmp：

```bash
cd /home/czx666/today_full_20260901/today_full
ls -lt camera_ws/records/ | head -25
```

查看最近一次结果（Python不会发送任何机器人命令）：

```bash
/usr/bin/python3 - <<'PY'
import json
from pathlib import Path
base = Path('/home/czx666/today_full_20260901/today_full/camera_ws/records')
files = list(base.glob('run_*.json'))
if not files:
    raise SystemExit('还没有run_*.json记录')
p = max(files, key=lambda x: x.stat().st_mtime)
data = json.loads(p.read_text())
print('记录：', p)
print(json.dumps(data.get('final', {}), ensure_ascii=False, indent=2))
PY
```

实验说明：`camera_ws/docs/experiments.md`；集成原理及接口：`camera_ws/docs/map_s1_grasp_integration.md`。
软件状态、MAP误差不等于独立物理测量；入盒也不能仅靠last_exit_code=0判定。

## 10．停止一次测试、关闭节点、重新开始

### 10.1 先停止运动（PC已加载第8节环境的终端）

物理遥控急停始终保留。软件停止：

```bash
ros2 topic pub --once /stop std_msgs/msg/Int8 '{data: 1}'
```

测试终端也可按`Ctrl+C`，程序会请求停车、关闭本轮控制器和安全门、结束录像。
先确认机器人实际停止，再做下面清理。抓取中的软件停止不会自动松爪或起立，也不等价于切断机械臂动力。

### 10.2 停止定位和相机

在PC终端D按`Ctrl+C`，关闭本轮定位/感知链。
在GO2相机终端C按`Ctrl+C`，关闭D435。
本轮若还是旧的后台相机启动方式，可在GO2终端执行：

```bash
pgrep -af '[r]ealsense2_camera_node'
pkill -INT -f '^/home/unitree/go2_d435_foxy_ws/install/realsense2_camera/lib/realsense2_camera/realsense2_camera_node( |$)'
```

该命令针对本目录的RealSense节点；不要对其他相机任务或S1视觉进程执行宽泛`pkill python`。
定位若也是旧后台启动，先在PC查对应launch PID：

```bash
pgrep -af '[r]os2 launch go2_science_perception d435_slam_target_validation.launch.py'
```

只对上面查出的本轮launch PID执行`kill -INT 实际PID`；不要复制历史PID或直接杀所有ROS节点。

### 10.3 最后才关闭机械臂服务

GO2停止、抓取任务结束且机械臂有支撑后，按第3节关闭S1服务；此后不再让GO2行走。
只想重新测导航、且S1仍正常ZERO_HOLD时，可以保留S1服务，不必每轮关开。

### 10.4 再次测试的顺序

停止本轮运动 → 结束日志/录像 → 需要时重启定位感知 → 把目标和机器人恢复到测试起点 → 确认S1 ZERO_HOLD → 第8节选一个入口。
不要直接从FAIL/FAULT继续发grasp；不要把失败位置当作新的已确认抓取位姿。

## 11．严格使用原稳定MAP分支（可选，与第8节二选一）

仅在确实要回退原稳定源码时执行。下面建立独立只读提交工作树，不修改稳定分支：

```bash
cd /home/czx666/today_full_20260901/today_full
git fetch origin
git rev-parse origin/feature/d435-perception-workspace
git worktree add --detach /tmp/go2_stable_map_test origin/feature/d435-perception-workspace
```

目录已存在时跳过worktree add，直接进入已有目录，先核对`git log -1 --oneline`。

```bash
cd /tmp/go2_stable_map_test
source /opt/ros/humble/setup.bash
source /home/czx666/today_full_20260901/today_full/deploy_ws/install/setup.bash
colcon --log-base /tmp/science_stable_log build --base-paths camera_ws/src \
  --build-base /tmp/science_stable_build \
  --install-base /tmp/science_stable_install \
  --packages-select go2_science_perception
```

第6节GO2上的D435仍按明确序列号启动。先停掉旧的PC定位链，在新PC终端启动稳定版本：

```bash
cd /tmp/go2_stable_map_test
source /opt/ros/humble/setup.bash
source /home/czx666/today_full_20260901/today_full/deploy_ws/install/setup.bash
source /tmp/science_stable_install/setup.bash
source camera_ws/scripts/science_approach_pc_env.bash
ros2 launch go2_science_perception d435_slam_target_validation.launch.py \
  remote_camera:=true use_rviz:=true
```

另一个PC终端运行稳定搜索接近【真实运动，不抓取】：

```bash
cd /tmp/go2_stable_map_test
if [ ! -e deploy_ws ]; then
  ln -s /home/czx666/today_full_20260901/today_full/deploy_ws deploy_ws
fi
export SCIENCE_APPROACH_OVERLAY=/tmp/science_stable_install/setup.bash
bash camera_ws/scripts/run_map_anchor_experiment.sh \
  --execute --search --search-distance 5 --search-timeout 60 --timeout 25
```

稳定工作树默认日志写在`/tmp/go2_stable_map_test/camera_ws/records`；保存到主仓库后再清理/tmp。
稳定分支没有S1完整任务入口；不能在这个目录运行`run_map_s1_task.sh`。

## 12．常见问题和恢复

### 集成目录或/tmp构建目录不见了

只有在`/tmp/go2_map_s1_integration_20261111`确实不存在时，才在PC重新取一份：

```bash
git clone --branch feature/map-approach-s1-grasp-integration --single-branch \
  https://github.com/Czx666666888888/GO2-Lidar-Itself.git \
  /tmp/go2_map_s1_integration_20261111
```

然后执行第4节链接/构建/同步脚本。原始数据仍在主仓库`camera_ws/records`。
此命令不会修复缺失的导航底包或安装GO2的S1交付包。

### 全新GO2尚未部署S1交付包（本机已经部署，不要覆盖）

PC复制原包：

```bash
scp -i /home/czx666/.ssh/id_ed25519_go2 \
  '/home/czx666/文档/xwechat_files/wxid_kt48eys7r67z22_c5b4/msg/file/2026-11/S1_arm_integration_20261111_auto_zero_hold_v1.zip' \
  unitree@192.168.123.18:/home/unitree/
```

GO2校验包；已部署目录存在时不要重复解压覆盖：

```bash
sha256sum /home/unitree/S1_arm_integration_20261111_auto_zero_hold_v1.zip
```

正确SHA256：`a7e08d875366dc777e2a41e19401f537fa929584c9c89ae6a0ceaea5aab6596e`。

```bash
/usr/bin/python3 - <<'PY'
from pathlib import Path
import hashlib
import zipfile
archive = Path('/home/unitree/S1_arm_integration_20261111_auto_zero_hold_v1.zip')
assert not Path('/home/unitree/s1-arm-control').exists(), '目录已存在，不覆盖'
assert hashlib.sha256(archive.read_bytes()).hexdigest() == 'a7e08d875366dc777e2a41e19401f537fa929584c9c89ae6a0ceaea5aab6596e'
with zipfile.ZipFile(archive) as z:
    z.extractall('/home/unitree')
PY
```

这只负责原包部署，不等于全新Jetson依赖安装完成。新环境按交付包`docs/TEAM_INTEGRATION.md`安装，
不要覆盖已验证的SDK/标定；本机继续使用现有S1 Python环境。

### SDK2段错误 / 起立被StopMove=-1挡住

最新完整任务入口已处理本机这两项问题。姿态进程专用环境如下，**不要全局写入.bashrc，也不要给相机/S1抓取环境一起套用**：

```bash
CYCLONEDDS_HOME=/home/unitree/cyclonedds_ws/install/cyclonedds
LD_LIBRARY_PATH=/home/unitree/cyclonedds_ws/install/cyclonedds/lib
```

只写变量不会发送任何动作。协调器会为姿态子进程自动设置它们。
最新程序只在确认趴下静止和ZERO_HOLD后直接起立；趴下前仍保留StopMove要求。

### localization jump / TF持续失败 / 地图位置数十米异常

停止本轮运动；保留日志；在GO2站稳后关闭PC定位链，再按第7节原参数重启。
不能靠放宽定位跳变阈值继续走。本轮曾在独立趴下/起立采样后遇到旧定位链失稳，因果未确定。

### 多次STOP_CONFIRM后没有接近

看`stop_confirm_attempts`、有效样本数、时间跨度、散布和是否真的静止。
原总预算6s、至少5个样本、至少0.6s跨度等条件仍保留；不要只看状态名就判断已经确认。

### 抓取停靠超时与卡滞

历史vx/wz版本最近一次位置误差约9.94cm、航向约8.64°，未达到当时2cm/0.05rad条件。
当前三自由度版本使用4cm/10°，已取消固定25s总任务时限；旧`--timeout 25`在S1入口仅保留兼容、不会限制停靠。
若新版本出现`odometry continuation limit`，是独立的视觉丢失ODOM 5m/25s约束；
若出现`grasp docking stalled`，是10s没有有效位姿/停稳进展。两者都需保留日志分析，不自动扩大保护。
完整新控制与抓取仍需实机验证，原S1模块继续自行检查可观测性和可达性。

## 2026-11-13 右侧物块横向净距调整（静态）

用户确认前后到位但右侧过近，要求机身再远离20～35cm；本轮取中值27.5cm。
集成停靠Y目标由S1右侧0.31m改为0.585m，机身停车位向左移0.275m；前后位置、
4cm/10°/0.5s、速度和S1动作链不变。几何校验允许停车目标在抓取参考区外，
原29～33cm抓取参考区保持，predicted_in_grasp_region=false不伪称可抓取。
默认DockSettings历史值保留，真实入口通过science_grasp_docking.yaml覆盖；计划输出同步。
新版仅定向软件检查，真实净距/机械臂可达性NOT VERIFIED；下一轮先用dock，不直接full。
命令：bash go2_test.sh --execute --mode dock（真实运动，本轮未执行）。

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

2026-11-13 用户要求恢复SEARCH速度：实际集成profile search_speed恢复0.25m/s，安全门max_speed恢复0.30m/s；微调最低0.10、停靠无25s时间上限及横向额外10cm保留。S1最近串口I/O错误，本轮重试仍保留硬件阻断。
