# 单类别YOLO方块接近接口

## 最新：YOLO已接入原full/S1编排

PC默认full/dock现已显式选择YOLO，共用本页V2权重、venv及模型hash。
`bash go2_test.sh --execute --mode full --view`在原6cm停靠/停稳后释放导航并执行
趴下→S1抓取→ZERO_HOLD→起立；外层时间入口完整转发显示和检测器参数。
本页approach模式继续保留为仅视觉接近10cm、不操作S1。后文“full保持原路径”
是此接入之前的历史说明，当前full感知已通过原launch参数切换YOLO，动作算法未改。
启动复用须匹配实际检测器和V2模型输出，禁止HSV/YOLO正式目标混用。
本轮54项相关软件检查/单包构建/安装一致性通过，无实机动作，物理全链路NOT VERIFIED。

## 入口修复：伴随主机重启后恢复对时

无相机时检测到>50ms时钟偏差，复用原sync_go2_clock.py交互恢复并重新严格检查。
已有相机时不跳改时钟，SSH/RTT异常也不触发对时。--view继续原参数转发，
画面在感知链建立后启动；真实接近/S1边界不变。本次13项相关模拟检查通过，未执行真机运动。

2026-11-14。开始/提交前核对远端：视觉分支abb83f5，实际集成分支caf6ab6；阅读两套实际源码，
不是依据旧文档推断。用户随后要求“先完成代码撰写”，因此本轮不连接设备、不启动实时预览或运动。

## 一键入口

从原PC主仓库运行；主入口把approach转给 `/tmp/go2_onekey_s1` 的实际集成实现，并传入
`GO2_VISION_ROOT` 指向主仓库的V2权重/venv。直接在集成目录运行时可显式设置此变量。
原full/dock入口保持原路径；新approach不进入S1/full Workflow，也不进入aarch64本机抓取入口。

```bash
# 只打印离线计划，不连接设备、不创建ROS节点。
bash go2_test.sh --mode approach

# 后续现场准备好后：RGB-D/Point-LIO/MAP/录制/显示，控制器dry_run，不启动gate、不arm。
bash go2_test.sh --mode approach --preview --view

# 本轮未执行！仅供以后用户明确授权、路线空旷、急停在手时的一次方块接近。
bash go2_test.sh --execute --mode approach --view

# 先Ctrl+C退出原会话，再回到原蓝色HSV流程；不是自动故障切换。
bash go2_test.sh --mode approach --preview --view --detector hsv
# 以后授权运动时同样可用 --execute --mode approach --detector hsv。
```

默认preview持续60秒，可用`--duration 120`调整；运动沿用45秒总时限，可用`--timeout`在60秒内指定。
默认伴随主机D435，使用原launch_go2_d435_foxy.sh与SSH所有者清理；可选`--local-camera`。
已有正式目标/速度发布者或定位链时拒绝启动重复链，不自动杀旧进程或切换检测器。
只复用现有相机时不负责停止外部相机；退出只清理本轮拥有的进程。无需启停或部署S1。
当前approach明确只支持PC，不冒充aarch64本机部署；full/dock原有本机路径不改。

YOLO使用现有V2：`vision_training/runs/science_targets_mixed_v2/baseline/weights/best.pt`；
SHA256 `e9bfab914e3412738b1ba9a45db24646a756ac75ab92abda8012aae3a37b46a1`，不下载替代权重。
CLI可显式指定本地`--model-path`/`--python-executable`；模型bytes须仍匹配此V2。
`--view`需要桌面与rqt_image_view；无桌面可去掉该选项，录像和日志仍保存。

## 接口与深度

- `yolo_detector`只配置`class_names=color_block`，即既有V2类别0；两类岩石不进入正式目标。
- RGB来自color/image_raw，深度来自aligned_depth_to_color/image_raw，内参来自color/camera_info。
  检查尺寸、RGB/Depth/CameraInfo optical frame一致；RGB-Depth≤50ms、RGB-CameraInfo≤100ms，
  非零时间戳、新鲜度300ms；源时间戳保留，不用收到时刻覆盖。
- 沿用框中心50% ROI：至少20个有效像素、有效比≥0.6、深度0.15–5m、P90-P10≤0.08m。
  在中位深度层内选一个**实际有效像素**，不直接投影检测框中心/深度空洞；用原CameraInfo畸变处理反投影。
  返回中位深度、所选像素、该像素深度、ROI范围、有效比/离散度及拒绝原因。
- 此点是可测表面的粗定位，不等同方块几何中心；未做中心补偿。当前安装TF仍沿用原参数，
  不伪造新的标定。实际ROI落在物体还是背景、实际位置误差必须由后续D435静态样本核验。
- `/science/yolo/detections`完整帧JSON绑定每个框的类别、confidence、ROI质量、相机XYZ、RGB时间戳、
  optical frame、Depth/CameraInfo时间戳及frame。原`/science/yolo_target_point`只作独立诊断，不接Manager。

新增`color_block_target_adapter`继承并复用原`camera_target_to_map`精确时间TF和MAP诊断输出。
完整帧中有可见方块深度无效时整帧不选，避免把“其他可测方块”冒充失深度的最近/已锁目标。
在同一源时间取得map←optical和map←camera_link：第一次选相机原点MAP XY最近的可测方块，
后续只更新其初始0.20m、当前0.15m范围的唯一空间匹配；不因另一个目标变近而切换。
任意两个可测方块MAP XY距离≤0.40m时拒绝该帧，保守避免关联区域重叠；本轮不宣称支持紧邻方块。
初始锁定来自第一次合格帧，之后仍必须通过原Manager及Approach的多帧稳定确认才能运动。
重选目标须结束并重新启动会话，不自动把缺失目标换成另一个框。

只发布选中目标的 `/science/target_coarse_point` 和原MAP流；相机点和MAP点均保留RGB源stamp，
Manager/Approach仍消费原接口。不是把多类/多框无身份PointStamped逐个混入单目标链。
转换节点原始PointStamped订阅在适配器内禁用，防止回环/重复转换；没有改原转换器实现。
查询不到精确时间TF可等待新数据，在300ms有效期内重试，超过就拒绝；不使用latest TF替代。
启动互斥选择HSV或YOLO；适配器还检查正式相机点/MAP点和YOLO输入的发布者数量，竞争时不更新。

## 接近和失检

复用实际集成分支的原`science_target_manager`、`science_target_approach`和`approach_control`；
其源码及science_target_approach.yaml未改，hash见evidence/color_block_interface/unchanged_integration_controls.json。
原5次/2秒、散布≤8cm的稳定锚点、视觉更新门限、Point-LIO位姿闭环、制动/超时/安全门不改。
终点仍是`stop_reference=camera_origin`、`camera_link`原点与目标MAP XY距离≤0.10m，忽略Z。
不使用抓取停车位、4cm停靠精调、模型前向脉冲，不传`--grasp-pose`，不启动机械臂、趴下或起立。

- 未建立有效稳定目标时，不启动运动；不把空检测/无效深度当成坐标零。
- 锁定后YOLO失检、深度/内参/同步无效、目标出界或歧义：适配器不发新点。
  原控制器视觉超时0.5秒后可转APPROACH_ODOM，用既有MAP锚点与Point-LIO闭环，
  **不是立即停车**；原ODOM上限25秒/累计路径2m以及总时限、定位/TF/反馈异常保护仍有效。
- 到位、保护故障和运行超时使用原STOP/FAULT及安全门停止路径。Ctrl+C让原实验先停止/解除arm，
  再清理本轮相机和定位；软件完成不是物理停稳证据，后续真机必须保留物理急停。

## 画面和数据交付

新录制开关可发布`/science/approach/debug_image`供rqt实时显示：YOLO框、ROI投影像素、
Manager ID、MAP目标/相机位置、原XY距离、状态和速度。YOLO原debug_image补充相机XYZ/深度拒绝原因。
YOLO同时提供原录制器兼容debug，但启动时不运行HSV，也不生成伪HSV mask。

每轮目录`camera_ws/records/color_block_<ns>/`保存manifest（代码SHA、模型hash、模式）、
相机/检测/TF进程日志、`map_positions.log`与`approach_status.log`。
preview录像在该目录；真实接近录像/最终run JSON仍沿用原Experiment的camera_ws/records命名。
`/science/color_block/selection`提供选中检测、相机XYZ、MAP XYZ、候选位置、初始锁定点和拒绝原因；
`/science/map_measurement_debug`记录实际转换的同源相机/MAP点；`/science/approach/status`提供
camera_distance_xy、target_map_anchor、状态与reason。原run JSON的flow_metrics记录最终MAP距离/停止原因；
实际物理距离必须另行测量，不能把该MAP指标当物理真值。

## 本轮验证边界

- 两分支各12项针对性软件检查通过（同一检查集，不累计宣称24个不同测试），包括目标锁定/拒绝切换、
  相邻目标拒绝、实测像素避开空洞、源stamp/frame保留、岩石过滤、深度无效不更新和原TF辅助检查。
- 实际集成分支感知单包colcon build通过（30.3秒）；指定venv的rclpy/cv_bridge/Ultralytics8.3.40导入通过。
- launch `--show-args`、离线入口计划、shell/Python语法检查通过；没有启动ROS运行节点或发送硬件命令。
  第一次参数检查早于构建完成而失败，随后因默认ROS日志目录只读失败，指定/tmp日志后解析通过。
- 原控制/Manager/转换器/0.10m配置未改，full/dock入口新增模式之外的原逻辑保留。
- D435连续RGB/Depth/CameraInfo同步、真实深度ROI位置误差、GPU实时吞吐、实时画面、相机/MAP实测坐标、
  实际到位距离/停止原因、单方块及多方块真机运行：全部 **NOT VERIFIED**。
- 按用户最新要求，本轮只交代码。后续先获授权做静态预览并确认ROI与量测误差，再单方块，
  用户确认单方块实际停止后再测试间隔充分的多方块最近目标选择。
