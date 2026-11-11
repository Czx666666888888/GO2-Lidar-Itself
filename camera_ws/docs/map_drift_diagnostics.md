# 固定物块 MAP 一致性诊断（2026-11-08）

本工具不发布运动命令。禁止同时启动 APPROACH、FAR、WP5；B/C 由现场用户遥控。
外参仍为 PROVISIONAL / NOT CALIBRATED。MAP 内部一致性不等于绝对定位精度。

## 记录与证据边界

- 原检测 `/science/detection_debug`：原始整数 RGB/Depth stamp、像素中心、
  有效深度点数、中位深度、camera XYZ、前端接受/拒绝原因。
- `/science/map_measurement_debug`：原测量时间、查询时间、实际使用的 camera→MAP
  平移/四元数、TF stamp、变换前后 XYZ；不更换为 latest TF。
- `/state_estimation`：原始 Point-LIO Odometry（含 frame、stamp、位置、姿态）。
- `/science/target_tracks`：ID、原始/滤波位置、源时间及 manager 一对一关联每对
  ACCEPT/REJECT、距离、旧/新位置、半径/竞争原因。
- controller 原有状态增加 association_diagnostics，仅记录实际分支；本实验不启动
  controller，因此该字段不作为 A/B/C 的实机控制器重关联证据。
- JSONL 保留全部原始事件。分析器只在原始 stamp/camera XYZ 唯一对应时连接检测，
  不用消息到达时间代替测量时间；ID 匹配缺失/歧义显式保留。
- Point-LIO 同测量时刻由前后最多100ms bracket插值：位置线性、四元数最短符号
  normalized lerp。原始 bracket 时间和frame保留；缺少 bracket 不用最近/latest 冒充。
- 视频复用现有 approach_video_recorder，可独立运行，不依赖控制器。

## 人工分阶段采集

先确认两主机对时、真实 RGB/Depth 连续、TF 成功及唯一固定物块。每阶段都用同一
SLAM 会话，避免 MAP 重置；B/C 不改变光照/安装/物块。A保持静止15秒；B缓慢左右
各20–30度；C约0.15m/s前进0.5–1m，可安全返回时返回标记。运动由用户完成并保留急停。

PC 环境：

```bash
source /opt/ros/humble/setup.bash
source deploy_ws/install/setup.bash
source camera_ws/install/setup.bash
source camera_ws/scripts/science_approach_pc_env.bash
ros2 launch go2_science_perception d435_slam_target_validation.launch.py remote_camera:=true use_rviz:=false
```

独立终端启动录像（远端相机须已启动）：

```bash
ros2 run go2_science_perception approach_video_recorder
```

另一个终端逐组记录（按实际阶段替换label和文件名；B/C延长seconds覆盖全程）：

```bash
python3 camera_ws/scripts/record_map_drift.py --label A --seconds 15 --output camera_ws/records/map_drift_A.jsonl
```

分析命令（每组独立目录；B/C必须指定A中固定参考点 `--reference X Y`）：

```bash
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/map-drift-mpl /usr/bin/python3 camera_ws/scripts/analyze_map_drift.py camera_ws/records/map_drift_A.jsonl --output camera_ws/analysis/map_drift_A
```

默认参考为第一份原始MAP观测。必须人工核实始终同一个物块，不能将其他候选混入。
CSV、summary.json和diagnostics.png保留测量/统计。多物体/SLAM重置应判该组无效。
当前分析器不自动判断根因，亦不将时间相关性当成因果。

## 已有失败数据审核

`approach_20261107_173154_217135.jsonl`：175份检测、80份含accepted目标。
RGB减Depth时差：全体最小-67.508992ms、中位0、最大0；accepted绝对时差P95
67.1925888ms、最大67.501312ms。旧时间戳经过double秒再转整数，含亚微秒舍入。
视频完整解码332/332；抽帧可见目标从画面上缘进入更完整视野，检测区域变化。
这支持排查表面取点变化，不足以证明错误物体关联或外参错误。

`/tmp/science_search_slam_retry.log`当前不存在；视频JSONL没有逐帧原始MAP点、完整
camera→MAP TF、Point-LIO位置/四元数、逐次track关联状态。因此不能恢复T88→T89/T90
的完整误差分解；原文档的0.23–0.39m保留为前轮记录，不能冒充本轮重算值。
现有run中的robot是控制周期下的转换后body XY/yaw，不能冒充源测量时刻Point-LIO姿态。

实机预检发现GO2时钟回到2022-09-08，PC为2026-11-08，NTP关闭；随后已恢复对时。
该问题已在正式采集前解决，不能反推昨天的时钟状态。

按record_ros_ns划分旧窗口（不是ID逐帧身份核实）：1794043923–3925锁定附近14份
检测/7个有效候选，中心v=8.71–11.17，median深度2.685m、面积296px、有效深度225点；
1794043931–3935恢复附近30份检测/31个有效候选，中心v中位125.32，median深度1.199m、
面积1298px、有效深度1084点。上缘截断和观测表面改变值得重点检验；物体相对相机距离
随运动改变本身正常，不能将两窗口深度差直接当作深度测量误差。31候选也不等于31物体。

必要验证：独立package构建通过；manager、MAP锚点、TF转换和粗检测现有相关测试
共36 passed；旧数据分析脚本运行通过；视频332帧解码通过。新字段已由正式A/B2/C实收连续数据验证；详见experiments.md最终结果。

## 本轮结果与重现

最终结论和受控条件限制见experiments.md的2026-11-08段。
原始大数据只留本机；提交的CSV/统计/图不等于外部真值。

B2/C用视频身份注释选择，示例（不按MAP误差阈值筛数据）：

```bash
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/map-drift-mpl /usr/bin/python3 camera_ws/scripts/analyze_map_drift.py camera_ws/records/map_drift_B2_20261108.jsonl --output camera_ws/analysis/map_drift_20261108/B2_selected --reference 0.9650134234919904 0.15301439430597485 --identity-csv camera_ws/analysis/map_drift_20261108/B2_identity.csv
PYTHONNOUSERSITE=1 /usr/bin/python3 camera_ws/scripts/summarize_map_drift.py
```

未来录像同时保存同帧无遮挡`*_raw.mp4`和带原检测覆盖的视频；原始视频不提交。
对时请在所有传感器启动前在PC交互运行`python3 camera_ws/scripts/sync_go2_clock.py`，
密码仅本机终端输入、不保存。脚本通过单次sudo内的READY握手后发送当前时间，
不把输入密码的等待算进时差。正式验证需同一SSH连接多次RTT边界，不用建连中点估计。
