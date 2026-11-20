# 2026-11-14 一键入口与执行记录

在任意目录使用同一句命令：

```bash
bash /home/czx666/today_full_20260901/today_full/go2_test.sh --execute
```

该命令会执行真实运动和机械臂动作。每轮应由操作者确认路线空旷、物理急停/监护和S1初始位/支撑。无参数仍只显示离线计划。

本轮只补主入口恢复：缺少 `/tmp/go2_onekey_s1` 时克隆集成分支；已有目录核验分支，创建并校验主仓库deploy_ws/records链接，再转入原集成入口。不覆盖主仓库或集成目录脏改动，不修改控制算法、阈值、串口绑定或失败恢复。

## 验证与实际结果

- shell语法、离线计划、diff检查通过；隔离模拟确认缺失目录恢复、重复进入复用和错误链接阻断，未访问硬件。
- 集成分支 `camera_ws/test/test_startup_interfaces.py`：10 passed；本轮没有运行全仓测试。
- 实际远端集成提交：`ddbe7e6c9b233134ce4c63ab657dc54ff648d336`。
- 首次PTY启动在本机sudo验证处中断，未改变校时服务、未启动动作；随后打开本机终端供操作者输入密码，继续同一入口。
- 实际单包构建与时间映射准备通过，prepare_code_and_build=3.617s；connection_and_inventory=0.907s，FAILED。
- 只读复核：路由 `dev enp12s0 src 192.168.123.99`；SSH正常；D435 `213622070598` 与D455 `231622302209` 均可枚举。
- `/dev/ttyUSB0` 不存在，`/dev/serial/by-id`、`/dev/ttyUSB*`、`/dev/ttyACM*` 未列出设备。只证明当前枚举缺失，供电/线缆/驱动根因未知。
- 无旧PC速度控制进程命中，远端未发现S1保持/抓取进程。本轮未到S1使能、SEARCH或抓取，完整动作 **NOT VERIFIED**。
- 最终退出1；PC校时restore_exit=0、service_active_after=true，并用systemctl实读active。

## 本地原始证据

- `camera_ws/records/pc_time_1794646217247586234/report.json`：PTY中断。
- `camera_ws/records/pc_time_1794646235669773633/report.json`：实际执行退出与校时恢复。
- `camera_ws/records/full_1794646241909471210/manifest.json`、`startup_timing.json`：实际commit、模式和阶段结果。
- 集成目录 `.onekey/log/latest_build/go2_science_perception/`：单包构建日志。

记录为本地证据，不随Git分发。下一步由操作者检查S1供电/USB后复核枚举；恢复后重新授权新一轮，不自动重跑、不改用其他设备名。

## 同日用户授权重启及断网收尾

- 用户要求重新启动一次后，同一入口设备检查通过，S1保持、相机、IMU映射/SLAM/TF预检完成，进入SEARCH；停稳确认首轮超时，后轮1.900s确认6帧、散布22.10mm，进入APPROACH_TO_GRASP_POSE。
- 接近约8.5s后发生localization timeout，控制输出归零，gate记录独立停止、续租超时及DISARMED；最后可用定位的停车位误差269.48mm，不构成停靠通过。未趴下、抓取或起立。
- 实读PC有线enp12s0 DOWN、机器人路由落至代理；用户明确反馈远端网线脱落，并确认已急停、机身停稳。以此区分外部断链与控制算法结论，未据旧Sport速度认定停止失败。
- 对本轮入口发送正常中止，最终退出130；PC校时restore_exit=0、service_active_after=true，systemctl核对active。用户随后要求清理，复核PC运动/定位/感知进程均退出，并关闭剩余只读日志进程组。
- 远端断网，S1保持/相机退出状态NOT VERIFIED；没有关闭保持服务，也不声称远端清空。接回网线后原入口会检查已有状态再决定复用或阻断。
- 证据：`camera_ws/records/full_1794646336884820782/`（manifest/startup_timing/entry.log/entry.exit），`pc_time_1794646333393901403/report.json`；控制前缀`1794646352502943313`，状态`science_approach_1794646353297096826.jsonl`。
- 原始/叠加录像`approach_20261114_165233_329358{_raw,}.mp4`均实际完整解码432帧；保留本地，不删除。协调器未生成本轮最终run JSON，不伪造。

## 当前到位/趴下前逻辑只读审查

来源：本机集成ddbe7e6源码、安装文件逐字节一致性，以及随后生成的`science_approach_1794646752708710688.jsonl`；本次审查未启动运动、未改控制代码。

- 实际安装配置为S1目标XY=(0,0.41)m；此前状态文件中的0.585m与默认DockSettings的0.31m不能作为当前入口参数。S1基座vehicle XY=(-0.06,0)m，按现有坐标约定，目标位于机身后方6cm、右侧41cm。几何/外参仍为暂定。
- 搜索0.25m/s、5m/60s；停靠同时修正vx/vy/wz，保持搜索起始航向。平面速度模长上限0.30m/s、请求下限0.10m/s，Kp=1.5；制动及向量限加减速优先，故最终命令可低于下限。加速0.15m/s²、减速0.30m/s²、制动延迟0.30s；角速度为2.0倍角误差，上限0.30rad/s，误差≤10°时归零。
- 仅按距离计算且未触发额外实测速度制动/斜率限制时：50/20/10/8/5cm误差对应约0.300/0.251/0.147/0.120/0.072m/s。这是请求速度计算，不是实机保证。
- 到位需要平面误差≤4cm、yaw≤10°，且机身/相机位姿差分与Sport XY三种速度均≤0.05m/s，姿态差分和Sport yaw≤0.10rad/s，数据新鲜并持续0.5秒。进入4cm范围开始限减速，速度并非立即跳零；离开误差范围会清除停稳计时。
- STOP且grasp_docking_complete后，协调器清理控制器/gate，检查无竞争运动发布者、S1 ZERO_HOLD，再将Sport控制权交给SDK；实收速度检查及StopMove成功后才调用StandDown。
- 趴下反馈确认需body_height≤0.084m、mode=0、XY速度≤0.05m/s、yaw/gyro≤0.10rad/s，连续1秒且高度跨度≤1cm，最长20秒；不代表独立触地测量。
- 待决策风险：0.41m停车目标在原抓取参考区Y=0.29～0.33m之外；predicted_in_grasp_region和--docking-geometry-verified目前仅记录，不阻断full。本轮不擅自修改该行为或宣称机械臂可达。
- 待验证风险：未设停靠总时限、无进展超时及丢视觉时间上限；仍有5m失视觉累计路径预算和新鲜度/异常/急停保护。小误差下的速度下限、制动滞后与定位噪声是否共同导致反复修正，需要证据，不能仅凭代码判定根因。
- 最新日志停靠约57.7秒，曾误差11.05mm，但该时刻位姿差分速度0.109m/s，尚非停稳通过；最终失视觉累计路径约5.285m，报remaining docking distance exceeds odometry budget，末次位置误差79.55mm，未满足趴下条件。命令XY峰值0.30m/s、Sport XY峰值0.287m/s、位姿差分峰值0.342m/s，各自含义不同，不能声称真实速度严格≤0.30m/s。

## 0.20m/s版单次完整测试启动

- 用户重新确认网线/现场/急停/监护/S1初始位及支撑，授权完整流程一次。版本`cb99af0889feb8b7b85aaeb8644a27d46fcd88cd`，已安装dock_min_linear_speed=0.20。
- 启动前enp12s0 UP、192.168.123.99/24，机器人直连路由正确，无旧PC控制进程。
- 实际执行`bash go2_test.sh --execute`；构建缓存复用，设备清点阶段报`COMMUNICATION_ERROR: /dev/ttyUSB0 missing`，退出1。未进入本轮S1准备、SEARCH、趴下或抓取；不自动重试。
- 事后只读SSH枚举发现`/dev/ttyUSB1`及`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 -> ../../ttyUSB1`；不能仅据USB转换器名称认定是正确S1。
- 历史S1服务PID3107仍带`--dev /dev/ttyUSB0`；运行进程不证明串口通信正常。未终止保持服务、未打开第二SDK、未改串口绑定。重枚举原因及正确设备身份待验证。
- PC校时restore_exit=0、service_active_after=true，systemctl核对active；本轮PC入口/控制进程均已退出。动作效果NOT VERIFIED。
- 证据：`camera_ws/records/full_1794647520693040045/`内manifest/startup_timing/entry.log/entry.exit，`pc_time_1794647516992263154/report.json`。

## 用户授权S1安全重启到USB1

- 用户明确确认机身停稳、机械臂支撑/活动空间/急停，并允许停止旧保持、在新串口使能及原流程回零；本轮不授权GO2行走或抓取。
- 只读确认旧PID3107空闲busy=false、navigation_allowed=false、readiness_state=COMMUNICATION_ERROR（ttyUSB0不存在）。随后SIGTERM正常退出，等待/proc消失，不使用强杀；确认USB1无占用后才启动原服务。
- 原服务源码hash保护/离线预检保留；新PID17836以`--dev /dev/ttyUSB1 --execute`运行。启动实测AT_ZERO，故原流程仅使能并进入保持，没有执行非零位插值回零；未扩大起步窗口。
- 连续5次状态：ZERO_HOLD、busy=false、navigation_allowed=true、at_zero=true、communication_state=OK，六关节最大绝对零位误差约0.010872rad（约0.623°）。反馈读取时间递增；仍无独立电机包时间证据。
- pgrep/fuser复核仅PID17836持有USB1，无抓取任务。保持服务继续运行；机械臂物理状态待用户目视确认，不将软件就绪视为整机运动/抓取验收。
- 本地证据`camera_ws/records/s1_restart_20261114_usb1/`：restart.log、restart_once.py、verified_status.json；远端`/home/unitree/go2_map_s1/s1_hold_s1_restart_20261114_usb1.log`及同名startup状态JSON。重启脚本为本次授权执行证据，不是新增日常启动入口，不应直接重复执行。

## USB1保持就绪后用户要求的单次全流程

- 入口版本951c8f7，安装停靠请求下限0.20m/s；启动前有线UP、直连路由正确，无旧PC控制进程，S1 PID17836 USB1 ZERO_HOLD且navigation_allowed=true。本轮按同会话已确认现场条件和用户再次要求执行一次。
- 实际设备/原生雷达检查通过，复用健康S1；本轮启动D435与定位。运动前30秒探针收到RGB428、base407、转换cloud411、IMU500、odom407，但对齐深度0帧，camera_ok/slam_ok/body_tf/camera_tf/ready均false。camera.log持续RealSense Frames Timeout；图像失败底层根因未确认。
- 原入口试图重建定位一次，但spawn以排他创建同名slam.log触发FileExistsError/Errno17，退出1。此为启动器重建日志名问题，不将它混同相机无深度的底层原因；本轮未修改算法或自动重试。
- 未启动本轮速度控制器，未SEARCH、趴下或抓取，完整流程NOT VERIFIED。PC本轮控制/定位进程均退出，远端无相机驱动或抓取任务残留；S1 USB1保持继续，状态复核ZERO_HOLD。
- 校时restore_exit=0、service_active_after=true，systemctl实读active。
- 证据`camera_ws/records/full_1794648107565969451/`（manifest、startup_timing、pre_motion.json、camera.log、slam.log、entry.log、entry.exit），`pc_time_1794648102829190588/report.json`。

## 最新17:26～17:28一轮：趴下阻断审查

本节为用户要求核对最新日志的只读分析，不是新一次运行。实际最新full目录`full_1794648390211512595`、commit951c8f7、mode=full，全部运动前阶段通过；不得再引用前一轮0深度作为本轮失败原因。

- `science_approach_1794648405751513071.jsonl`顺序为IDLE→SEARCH→STOP_CONFIRM→LOCK_TARGET→APPROACH_TO_GRASP_POSE→FAULT，无到位STOP；未生成本轮s1_task姿态日志。
- 停稳确认4.400s成功、6帧散布10.51mm，确已进入停靠。停靠误差最小29.65mm，yaw4.71°，当时机身/相机位姿差分速度0.0640/0.0624m/s，均超过0.05m/s；只有几何误差达标不等于停稳。
- 678个停靠周期中24个位置/yaw同时达标，只有9个启动停稳计时，分散为7段；最长连续记录跨度约0.1秒，未达0.5秒。不能据差分速度独自区分实际残余运动和定位漂移，根因待控证据。
- 首次FAULT为17:28:09.958、停靠67.900s，原因`remaining docking distance exceeds odometry budget`。当时视觉年龄62.311s，失视觉累计路径4.992079m，5m预算剩7.921mm；位置误差54.714mm，要进入4cm容差至少还需14.714mm，因此原预算保护阻断。
- 此处累计路径含往返修正，不是机器人离目标5m。视觉观测未更新的根因与反复修正底层原因未确认，不擅归因于相机断流或速度下限。
- 最终run状态FAULT、grasp_docking_complete=false；最终快照停车位误差94.196mm。run中flow_metrics.final_map_xy_error_m=0.471859实际对应camera_distance_xy，不可当成停车位误差。
- `run_map_s1_task.py:121`只允许STOP且grasp_docking_complete=true交接姿态控制，本轮在此条件之外，未进入StandDown请求。本轮不属于StandDown RPC被拒绝。
- gate记录独立停止、StopMove和DISARMED；校时已恢复。仅日志证据，不据此新增物理停稳/抓取成功结论。
- 原始证据`run_1794648404980919978.json`、同前缀control/gate日志、上述science JSONL与`pc_time_1794648390162082973/report.json`，均在camera_ws/records。

## 17:37～17:39新版仍未趴下：4cm入口未达到

- 用户现场反馈位置可接受且已停止稳定；本节按现场停稳观察记录，不用定位差分否定物理观察。只读核对最新full_1794649020289164112，commit340fd13、full模式，使用新版精调/零速等待。
- 最新science_approach_1794649036065546131.jsonl有742个停靠周期，0个位置≤4cm，0个STOP_WAIT；最小误差58.357mm（停靠29.800s），所以没有进入停车等待/成功计时。6cm是已进入等待后的退出阈值，不是完成容差。
- 502个停靠周期的机身/相机位姿差分及Sport XY速度同时≤0.05m/s（仅核对平移必要条件，不代表全部停止条件通过）。首次FAULT前一周期74.200s，机身/相机差分0.00414/0.00391m/s、Sport XY近0，但停车位误差91.524mm，因此仍在FINE_CONTROL。
- 同周期原始请求vx=0.07408、vy=0.03021m/s（合0.08），Sport反馈近零；这支持后续核验低速响应/定位漂移，不能单凭日志确定低速死区、运动失效或真实定位误差。
- 首次FAULT停靠74.300s，失视觉68.838s，累计路径4.946584m，预算剩53.416mm；误差93.649mm，到4cm边界至少需53.649mm，触发remaining docking distance exceeds odometry budget。
- 最终FAULT/complete=false，未进入StandDown（无本轮姿态日志）。最终快照误差104.668mm，不能与最小58.357mm混淆。PC校时恢复成功。
- 当前成功条件仍4cm/10°，原多路平移≤0.05m/s、姿态差分/Sport yaw≤0.10rad/s，里程计/相机TF≤0.4s、Sport≤0.5s新鲜，连续0.5s；随后关闭导航、排除竞争运动发布者、核实S1 ZERO_HOLD及SDK控制权/新鲜状态/StopMove成功，才可StandDown。趴下后高度≤0.084m是动作反馈条件，不是发起趴下前的位置门槛。
- 本轮仅分析，未改容差/控制、未重新运动。新版未覆盖未进入4cm前的停滞，真实停车参考与低速响应仍需验证。证据run_1794649035291469190.json、上述JSONL及full/pc_time_1794649020237955631。
