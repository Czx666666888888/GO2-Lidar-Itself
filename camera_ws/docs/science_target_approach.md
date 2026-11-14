# 单目标视觉 / MAP 锚点混合闭环

## 2026-11-14 dock模式补充

仅grasp docking配置启用窗口里程、LIO/Sport矛盾停车、STOP_WAIT时间窗稳定性、
视觉3秒过期停车，以及0.20正向短步有效速度。普通MAP/SEARCH与STOP_CONFIRM逻辑不变。
原始/窗口里程及证据矛盾写odometry_evidence，停车阻塞和退出原因写原控制JSONL。
用户新批准位置<=6cm，航向10度和实际停稳0.5秒不变；短步已开启但MOVE需要真实制动profile。


以`858f75a`为基线，不启动FAR、WP5、localPlanner、pathFollower。相机、Point-LIO、
蓝色检测/蓝墙过滤、map变换和visited继续复用。当前成功参数在
`config/science_target_approach.yaml`：最高命令0.30 m/s、kp_linear=1.0、
`stop_reference=camera_origin`、camera_link原点XY容差0.10 m、allow_odom=true。

## 当前行为

- 最近2 s内至少5个新鲜稳定样本的中位数建立MAP锚点，散布阈值0.08 m。
- 锁定后不随selected或ID刷新切换目标，不因track删除/合并或视觉间断删除锚点。
  新观测须靠近初始锚点0.20 m、当前锚点0.15 m，唯一空间匹配才以alpha=0.15小幅修正。
  歧义/跳变拒绝更新。visited人工操作仍可使当前任务停止。
- 使用`/base_state_estimation`原始时间戳查询完整map->camera_link TF，实时计算相机原点；
  包含真实TF中的旋转和前置安装偏移，不把机身中心当相机，不直接加0.36 m。
  `distance_xy=hypot(xt-xc,yt-yc)`，停车忽略Z。旧0.80 m机身standoff不参与该模式。
- 目标观测新鲜时APPROACH_VISUAL，过期时APPROACH_ODOM；后者保留锚点并用实时位姿
  闭环，默认上限25 s / 实测累计里程计路径2.0 m，不用速度乘时间推算位置。
- 朝向误差大时先转向，距离比例控制及制动上界连续减速，0.10 m内锁存STOP。
  越过目标平面但未进入容差时FAULT，避免绕圈追逐；STOP/FAULT都归零并经gate停止。
- 保留定位/TF/时间戳/状态超时、姿态与位置跳变、原始时间倒退、时钟回退及速度发布者
  竞争保护。future_tolerance=0.10 s适配已测得约50–60 ms超前；不替换原始stamp。
  上游没有map epoch消息；重置地图须停止并重新建立锚点，连续小漂移不能全部自动识别。
- gate仍是唯一持续Sport输出者。min_speed=0，保留lease、加速度、超时和急停。
  控制器确认前和运行中核对gate参数及唯一可发现本机Sport输出者，不直接发布Sport请求。
  正常退出发送停止；控制进程死亡依赖仍在运行的gate命令超时，物理效果不能仅靠源码认定。

## 视频与日志

`approach_video_recorder`直接订阅实际D435 RGB，不是RViz/模拟。复用检测器发布的
`/science/detection_debug`轮廓、面积、中心点及`/science/detection_mask`；叠加锁定ID、
锚点、相机MAP位置、XY距离、vx/wz、状态和原图时间戳。ODOM期间显示
`TARGET LOST — MAP ODOM CONTROL`，HSV mask小窗口保留。

`DETECT=VISIBLE/LOST`表示RGB检测；APPROACH_ODOM表示锚点MAP观测过期，两者可能不同。
JSONL保存每个视频帧索引和RGB stamp、检测及10Hz控制数据，可判断是否出视野、阈值
过滤或时间间断。录像健康不满足会拒绝运动，运行中健康过期则停止。视频停车后继续
保留3 s，结束封装MP4。输出默认`camera_ws/records/approach_<时间>.mp4`及同名JSONL。
视频固定15fps，实际到帧时间以JSONL和画面原stamp为准。记录文件本机保留、Git忽略。

控制JSONL另记Sport原始速度、mode/gait/error、位姿差分速度及锚点更新/拒绝次数。
ETA来自实测朝目标速度；静止漂移可能产生伪速度，不以命令速度充当实测速度。

## 本机启动

GO2相机仍在`192.168.123.18`，独立Foxy wrapper位于
`/home/unitree/go2_d435_foxy_ws`。已有`launch_go2_d435_foxy.sh`仅采集相机并统一话题名；
PC采用`science_approach_pc_env.bash`配置peer。两机必须先对时，NTP active不等于同步。
本轮发现Orin时钟回退以及旧SLAM发散，重建只读SLAM后才开始锚点实验。

本轮使用的独立构建入口：

```bash
cd /home/czx666/today_full_20260901/today_full
source /opt/ros/humble/setup.bash
source deploy_ws/install/setup.bash
colcon --log-base /tmp/science_anchor_log build --base-paths camera_ws/src \
  --build-base /tmp/science_anchor_build --install-base /tmp/science_anchor_install \
  --packages-select go2_science_perception
source /tmp/science_anchor_install/setup.bash
source camera_ws/scripts/science_approach_pc_env.bash
```

先确保只有一套现有相机、SLAM/TF/manager/检测链；必要时只读启动：

```bash
ros2 launch go2_science_perception d435_slam_target_validation.launch.py \
  remote_camera:=true use_rviz:=false
```

只预览和录像（不启动gate、不发raw、不运动）：

```bash
ros2 launch go2_science_perception science_target_approach.launch.py
```

用户要求直接运行时，一条显式执行命令即为人工启动，不再弹额外确认：

```bash
export SCIENCE_APPROACH_OVERLAY=/tmp/science_anchor_install/setup.bash
bash camera_ws/scripts/run_map_anchor_experiment.sh --execute --timeout 15
```

需用户已确认单物块身份、平整无障碍、最终机身/腿/相机无干涉、运动模式、监护和物理
急停可用。程序等待录像和稳定候选，自动锁定后经原gate启动一次，STOP/FAULT后结束，
不自动复位或重复运行。可加`--target-xy X Y`限制为已确认MAP位置0.08 m内的候选。
默认打开独立GNOME日志终端；亦可单独运行：

```bash
bash camera_ws/scripts/watch_approach_logs.sh
```

该窗口只订阅状态，显示命令和实测反馈，历史记录有明确标记。包装入口整理自实测
监督逻辑，完成语法和--help检查；没有为包装入口再额外运行机器人。

保留原人工服务接口供调试：`/science/approach/confirm_target_id` Int32只锁定相机模式
目标，`/science/approach/start` Empty才开始；reset Empty清空终态和旧观测，不能自动
武装。`/stop` Int8=1立即停止。不得同时运行预览、实机两个controller或完整导航脚本。

## 本轮验收

kp=0.5轮在约34.2 cm处超时，用户实测35 cm；后半段命令约0.15而Sport反馈接近0。
提高kp=1.0而不抬升gate最低速度后，初始1.55 m，约7.83 s进入STOP，MAP误差8.3 cm；
停车后最后9.4 cm，用户确认实物约10 cm处停住且效果良好。本场景粗验收通过，
不是正式标定或多次重复精度证明。Sport反馈峰值0.316 m/s，实际速度严格不超过0.30
仍不能据命令限速认定。

成功录像：`camera_ws/records/approach_20261106_210841_496906.mp4`（204/204帧完整解码），
对应同名JSONL及`run_1793970520470079235.json`。完整失败/成功证据和边界见experiments.md。

## 2026-11-07 — 移动搜索自动接近

```bash
export SCIENCE_APPROACH_OVERLAY=/tmp/science_search_host_install/setup.bash
bash camera_ws/scripts/run_map_anchor_experiment.sh --execute --search \
  --search-distance 1.0 --search-timeout 20 --timeout 25
```

`--search`在原接近配置之上加载`science_target_search.yaml`。沿启动时机身MAP朝向走
1.0 m（默认），SEARCH命令0.20 m/s（用户在前三轮后要求由0.15提高），使用实时Point-LIO位置计算路线投影进度及横向误差；
超出0.20 m横向边界、搜索超时或路线结束均停车。路线无障碍由现场操作者确认，程序没有
避障功能。`--search-distance`允许(0,2] m，`--search-timeout`允许(0,30] s。

当前流程为SEARCH → STOP_CONFIRM → 既有APPROACH → STOP，目标为5cm蓝色立方体。
SEARCH沿用0.20m/s。源时间戳和原始MAP XYZ与`/science/map_measurement_debug`精确匹配后，
读取相机3D欧氏距离；距离≤1.8m且同一唯一候选连续3份不同源观测才请求停车。
更远的MAP点仅作提示；缺少匹配的相机量测时不触发。HSV和Point-LIO不变。

STOP_CONFIRM每周期输出零速度，不发送会锁存终止任务的`/stop`。同时要求Sport XY速度、
位姿窗口速度和相机位姿窗口速度≤0.05m/s，Sport及位姿yaw变化率≤0.10rad/s，连续
0.20秒满足后标记停稳。只有源测量时刻晚于停稳时刻的全新观测参与确认；再次运动会
清空样本并重做停稳判断。停稳后的1.0秒窗口至少5份不同源时间戳，源时间跨度≥0.60秒，
最大两点XY散布严格<0.08m，才取中位数锚点。接近之前还要求manager已CONFIRMED。

确认样本必须唯一匹配停车触发时冻结的空间种子，距离不超过既有0.15m跳变门槛及
0.20m关联半径；不能换成任意稳定蓝色区域。缺失、歧义或间隔过大不确认。
停车确认总时限6.0秒（包含制动/停稳/采样；用户在2秒及4秒实测后指定6秒），超时放弃该次候选；保留原搜索起点、路线及
总搜索时限，恢复SEARCH并冷却0.75秒，冷却后重新收集新源观测。不会用旧样本反复停车。

同一个controller依次完成全部阶段，唯一raw发布者；确认成功的切换周期输出零速度，
下一周期复用既有ALIGN/APPROACH_VISUAL/APPROACH_ODOM（LOCK_TARGET仅为交接状态）。
锁定后原平滑修正、0.30m/s命令上限、相机MAP XY≤0.10m停车、视觉丢失Point-LIO续航、
异常拒绝及原ODOM上限均不变，不做大跳变自动重新锁定，不启动FAR/WP5。

JSONL/run的flow_metrics记录首次有效相机检测距离、请求停车/停稳距离、停稳耗时、
静止采样耗时/散布/帧数、每次确认结果、视觉丢失次数及最终MAP误差/STOP或FAULT原因。
物理距离仍需用户实测回填，不能以MAP内部误差代替。视频显示STOP_CONFIRM和样本数。

SEARCH配置单独调整比例增益与制动延迟，旧人工接近配置不变。仍无最低速度抬升。
实际机器人低速/转向响应与MAP误差须看现场记录，命令上限不等于实际速度保证。

实际RGB视频显示SEARCH、当前MAP观测和相机位置，锁定后显示锚点与XY误差。JSONL
记录原始观测时间、首次MAP观测、确认时间/耗时、窗口样本数/最大两点散布、切换事件、
命令和Sport反馈、锚点更新及停车原因。录像中的检测日志另外保留首个RGB检测时间。
暗光识别沿用用户已实测结论，本轮不重复暗光验收、不调整HSV。不启动FAR/WP5或局部规划。


### 2026-11-08收工版本

当前STOP_CONFIRM总时限6秒。用户要求明天再测；6秒版尚未实机验证。
2秒版首轮（初始可见）用户实测约9cm到达；从不可见开始的后续测试有前端无候选、
停稳判据未及时满足的问题，4秒版亦超时，不能宣称重复成功率已提高。
明天先检查USB连接、对时、固定安装、物块/路线和急停，再运行上述`--execute --search`
入口。没有自动定时启动，不放宽停稳速度或坐标散布条件。原始视频与大日志仅留本机。

### 2026-11-09 远目标参数

用户明确要求SEARCH搜索路线5 m，并确认搜索速度为0.25 m/s（原输入2.5为口误）。
SEARCH YAML和执行入口默认路线现为5 m、总搜索时限60 s；入口允许距离(0,5] m、
时限(0,60] s。此前命令显式指定的1 m/20 s仍会覆盖新默认。
STOP_CONFIRM总预算保持6 s，SEARCH配置单独覆盖失视觉ODOM路径上限为5 m，
ODOM时间上限仍25 s，其他关联、停稳、散布及0.30 m/s安全门上限不变。
搜索距离从本轮启动位置沿启动航向计算，无目标时到达路线/时限即停车，无避障。

```bash
export SCIENCE_APPROACH_OVERLAY=/tmp/science_5m_install/setup.bash
bash camera_ws/scripts/run_map_anchor_experiment.sh --execute --search \
  --search-distance 5.0 --search-timeout 60 --timeout 25
```

本机临时overlay需先完成对应build；真实执行须具备已确认的路线、监护和物理急停。
实测结果另见experiments.md，不将参数修改视为6秒确认或5 m路径验收通过。
