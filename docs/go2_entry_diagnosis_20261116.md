# 2026-11-16 一键入口只读诊断

来源：本地记录、入口源码、主机进程快照及一次NX只读SSH。未执行full、部署、重启或运动。

- 实际集成版本5e818cf9aa35a127882e82b021a42693b660f1f2。
- full_1794817937121182678：准备/构建2.943秒、设备检查0.863秒、S1阶段5.058秒；existing探测3.784秒与S1并行，不能直接相加。
- existing.json：3秒收5帧点云，PC约1.702Hz，LidarState报告1.778Hz、cloud_packet_loss_rate=91.713，IMU样本500。native_ok=false，按入口源码在camera_clock_and_chain之前阻断；没有启动本轮相机/SLAM/运动。不将节点/阶段OK等同于健康通过。
- 原生雷达要求PC及状态频率均至少10Hz、状态loss不大于10、连续/新鲜数据及点时间跨度等条件。两端所见频率均低，不能仅归因PC DDS或YOLO；底层根因Unknown / needs verification。
- pc_time_1794817902550024197/report.json记录任务exit=1、was_active=false、full_clock_diagnostic=false、service_active_after=false。此轮没有40秒完整时钟采样，也没有由包装器停止原本活跃的校时服务。
- 主机进程快照曾见pc_time_check等待systemctl is-active --quiet systemd-timesyncd，子进程已运行20秒；源码的systemctl/timedatectl查询未设timeout。之后包装器退出。查询为何延迟仍未知，但无界查询可拖慢启动/收尾。当前报告只在finally末尾落盘，异常详情也未完整持久化。
- 前一轮full_1794817831151026809只记录prepare_code_and_build在3.698秒FAILED，manifest尚无detector信息。没有持久化异常堆栈，不能凭这份记录确定具体错误。
- 补充5秒只读native探测无点云/IMU/状态，报告/tmp/go2_entry_native_check_20261116.json。随后NX uptime显示up 0 min，S1 status报socket FileNotFoundError，未发现所查雷达/机械臂/相机进程；为后续现场快照，不能反推前一次失败时NX已经重启。当前真实就绪NOT VERIFIED。
- 有线路由实读为enp12s0 / 192.168.123.99，未走Wi-Fi/代理。
- 另见参数传递缺陷：pc_time_check.py choices仅right/left，default=right，并总传--grasp-side right；go2_full_test.py虽默认full为auto，实际常规入口被包装器覆盖。上一轮回答所称实际默认自动左右选择不准确。尚未修改。

下一步：先在NX稳定启动后只读确认原生雷达连续数据、原S1服务及设备状态；再修正包装器有界查询/异常记录和auto参数透传并离线验证。未获得新动作授权不重跑full，不放宽雷达保护。


## 17:42～17:45 最新一键入口“最后一步”只读复核

来源：本地full/pc_time记录、00983d0实际入口源码、PC进程/路由快照及17:47～17:48 NX只读SSH。
未运行full、启动相机、发动作、重启服务或修改生产代码。

- 17:42:55 full_1794822175800696372：prepare_code_and_build在2.963秒FAILED，未到最后就绪检查；本轮未取得该次异常详情。
- 17:43:04 full_1794822184322235294：final_live_readiness在30.921秒FAILED。RGB415/Depth400，base/cloud均0、slam_ok=false；slam.log明确launch加载异常TypeError: 'int' object is not callable及格式尝试的SyntaxError。不能凭后者断言XML源码确有语法错误，底层Python/launch触发原因待核验。
- 17:44:00 full_1794822240725727069：就绪9.962秒通过，曾进入SEARCH；1.418秒后控制记录首次FAULT/localization timeout，odom/camera年龄0.478秒>0.4门槛，Sport年龄0.433秒也同步增大。后续大量CycloneDDS UDP write retcode -1。表明多路数据连续性中断，但无法仅凭这些日志判定网线断开、接口变化或DDS具体根因。stop_confirm_attempts为空，未进入STOP_CONFIRM；没有本轮s1_task日志。
- 17:45:09 full_1794822309700138714：当前最新，manifest=00983d0/hsv/full。准备1.646秒、设备1.127秒、S1阶段10.506秒、相机/定位启动0.590秒；final_live_readiness在30.950秒FAILED。pre_motion记录SLAM/body TF正常、base407/cloud411/odom407，RGB60且没有Depth计数，detector0、camera_ok=false/camera_tf=false，ready=false。
- 17:45:26 D435日志先报RT IC2 Config error和Hardware Error，驱动Performing Hardware Reset，随后UVCIOC_CTRL_QUERY Protocol error与VIDIOC_QBUF No such device（统计9145条）。随后重新启动流不能证明连续RGB-D恢复，预检仍失败。直接原因是相机链路失效/重置期间没有有效连续RGB-D及检测，不是执行到了抓取后收尾。
- 最新pc_time报告elapsed=45.384秒、localization_exit=1、restore_exit=0、service_active_after=true；时间包装器已退出并恢复服务。此次PC服务初始查询0.027秒，不应沿用旧systemd查询卡住结论。
- 当前PC未发现go2_full_test/pc_time_check/SEARCH控制器；有线enp12s0 UP且路由192.168.123.18 dev enp12s0 src192.168.123.99。NX SSH可达，lsusb可见0b07/0b5c两台相机及HL340串口，S1原服务PID2734保留；枚举不等于稳定采流/当前零位或物理状态验证。
- NX dmesg普通读取被拒绝，sudo -n需要密码，未获得内核证据；RT IC2/USB重置具体硬件、电气、固件或驱动根因Unknown / needs verification。没有启动新采流或自动重试动作。

结论：终端所谓最后一步是统一30秒运动前就绪等待，最近几次在不同数据条件下失败；最新已经退出，非仍在无限等待。先做D435稳定RGB-D及前轮多路数据中断的独立诊断，不放宽保护。


## 18:02无运动与18:05相机枚举复核

来源full_1794823330453297664、pc_time_1794823330380174755及18:05:52 NX只读SSH。
实际45e6b2f，原日志重建修复已生效：存在slam.log/slam_2.log；两次pre_motion均正常完成30秒采样，
final_live_readiness总63.865秒FAILED。第一次RGB132但末次源年龄2.396秒，Depth0/检测0；
第二次RGB0/Depth0/检测0。两轮cloud412、odom/base408、IMU500且这些流fresh=true，
body_tf/camera_tf均false，ready=false。未生成控制器/gate/抓取日志，入口尚未进入SEARCH；
源码顺序证实失败发生在运动任务启动之前。当前PC入口/控制器/安全门均未发现。

相机日志仅确认D435枚举和驱动Up，此轮未见Hardware Reset或No such device，不能套用17:45窗口结论。
第一份slam.log另有source /opt/ros/humble时TypeError（list_iterator作为in左操作数），随后节点仍启动；
其与数据异常的因果未确认。base_odom的重复shutdown异常位于停止收尾，非初始阻断原因。
PC校时恢复成功，任务exit1，总71.053秒，非此次校时查询卡住。

18:05 NX当前可枚举D435(8086:0b07)和D455(8086:0b5c)，均在同一Genesys USB3 Hub上、5000M，
当前相机驱动已随任务退出，S1原PID2734保留。D435设备号由17:47快照Dev6变为Dev8，
这是两次快照间重新枚举的证据，可能由重置或插拔导致，不能确定时刻或唯一根因；D455仍Dev5。
当前枚举存在不等于采流稳定，也不能仅凭PC未收到Depth判断是NX采集、ROS发布还是DDS传输故障。
下一步需分段验证NX本机RGB-D产出和PC接收；本轮未启动相机采流或运动，不修改算法/门槛。
