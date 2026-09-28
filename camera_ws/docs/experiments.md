# D435 Experiment Record

## 2026-09-28 — Environment and USB Enumeration

- **Scope:** Read-only host/package/device inspection; no camera streaming and no robot nodes.
- **Host:** Ubuntu 22.04 kernel `6.8.0-138-generic`, ROS 2 Humble installation present.
- **Device evidence:** `lsusb` reported `8086:0b07 Intel Corp. RealSense D435`; udev reported `serial redacted` and `uvcvideo` binding.
- **Device nodes:** `/dev/video0` through `/dev/video7` and `/dev/media0` through `/dev/media2` existed; several nodes were identified by udev as D435 endpoints.
- **SDK:** no installed `librealsense2*` package; RealSense CLI tools absent.
- **ROS wrapper:** `realsense2_camera` and `realsense2_description` not found by `ros2 pkg prefix`; Debian packages not installed.
- **RGB stream:** `NOT VERIFIED`.
- **Depth stream:** `NOT VERIFIED`.
- **Color CameraInfo:** `NOT VERIFIED`.
- **Depth CameraInfo:** `NOT VERIFIED`.
- **Aligned depth to color:** `NOT VERIFIED`.
- **Camera TF frames:** `NOT VERIFIED`.
- **Interpretation boundary:** USB enumeration proves that the host can identify a D435 device. It does not prove librealsense access, ROS publication, valid images, sustained rates or depth accuracy.

## 2026-09-28 — Workspace Static Verification

- `colcon build --symlink-install`: succeeded for `go2_science_perception`.
- `colcon test --packages-select go2_science_perception`: 4 passed, 0 failed, 0 errors, 0 skipped; two non-failing Python entry-point deprecation warnings were reported by flake8 tooling.
- Installed executables: `camera_diagnostics`, `camera_info_inspector`.
- Both executables reached their startup log in a bounded 3-second run and were then stopped by `timeout` (exit 124 as expected). The restricted execution context also reported DDS UDP/netlink permission errors, so this is process-start evidence only, not ROS graph or message evidence.
- Python compile, package XML and both YAML files: passed static parsing.
- Full `camera_test.launch.py --show-args`: blocked exactly because `realsense2_camera` is not installed.
- `rosdep check`: reported missing `ros-humble-realsense2-camera` and `ros-humble-realsense2-description`; it also could not resolve the `ament_python` rosdep key in the current local rosdep database. The package nevertheless built with the installed Humble environment.
- Existing navigation source under `code/`: unchanged.
- Hardware stream verification: `NOT VERIFIED`.

## Bring-up Plan (completed below)

After explicit approval to install the official Humble packages, the planned steps were:

1. Record exact installed package versions and `rs-enumerate-devices` output.
2. Build this workspace.
3. Launch `camera_test.launch.py` without any GO2/navigation process.
4. Capture at least 30 seconds of diagnostics for all required streams.
5. Record actual topic names/types, frame IDs, resolution, encodings and rates.
6. Record color CameraInfo values and inspect `/tf_static`.

No target detection, map transform or robot motion belongs in this experiment.

## 2026-09-28 — D435 ROS2 Bring-up

- **Scope:** D435、官方 ROS wrapper 和 `camera_ws` 只读验证；未启动 GO2 或导航节点。
- **Packages:** `ros-humble-librealsense2` 2.58.4、`ros-humble-realsense2-camera` 4.58.4、`ros-humble-realsense2-camera-msgs` 4.58.4、`ros-humble-realsense2-description` 4.58.4，来自 ROS 2 Ubuntu Jammy 官方 APT 仓库。
- **Device:** Intel RealSense D435，serial redacted，firmware `5.12.7.150`，USB descriptor `3.2`；Stereo Module 和 RGB Camera 均由 `rs-enumerate-devices` 枚举。
- **Wrapper:** node `/camera/camera`；启动 Depth Z16 640x480x30 与 Color RGB8 640x480x30，启用 aligned depth，禁用 pointcloud/infra/IMU streams。
- **Topics:** `/camera/camera/color/image_raw`、`/camera/camera/depth/image_rect_raw`、`/camera/camera/aligned_depth_to_color/image_raw`、相应 color/depth/aligned CameraInfo、metadata、extrinsics 和 `/tf_static` 均实测存在。
- **RGB:** `sensor_msgs/msg/Image`，640x480，`rgb8`，frame `camera_color_optical_frame`。视觉抽样为有效非空画面。
- **Raw depth:** `sensor_msgs/msg/Image`，640x480，`16UC1`，frame `camera_depth_optical_frame`。
- **Aligned depth:** `sensor_msgs/msg/Image`，640x480，`16UC1`，frame `camera_color_optical_frame`。抽样非零像素比例约 23.3%，有效值约 154–12107 mm；与 RGB 尺寸/frame 兼容且视觉结构大致对应，但未完成定量像素对齐或精度标定。
- **Rates:** wrapper profile 为 30 Hz，color metadata 约 30.0 Hz，color/depth CameraInfo 约 30 Hz。大图像经 Python ROS 订阅时明显较低；完整 launch 长时累计约 RGB 18 Hz、aligned depth 8 Hz，aligned depth 曾短暂超过 2 秒未更新后恢复。因此“数据可用”已验证，“端到端持续 30 Hz”未验证。
- **Color CameraInfo:** 640x480、`plumb_bob`、`fx=608.898193359375`、`fy=608.2466430664062`、`cx=320.7738952636719`、`cy=243.91554260253906`、frame `camera_color_optical_frame`。
- **Depth CameraInfo:** 640x480、`plumb_bob`、`fx=fy=386.96484375`、`cx=318.3269958496094`、`cy=235.739501953125`、frame `camera_depth_optical_frame`。
- **Frames:** `camera_link -> camera_depth_frame -> camera_depth_optical_frame` 与 `camera_link -> camera_color_frame -> camera_color_optical_frame` 经 `/tf_static` 和 TF2 查询确认。`tf_publish_rate=0.0`，未观察到 wrapper 动态 `/tf`。
- **Workspace diagnostics:** 四类目标流均收到消息并多次同时报告 fresh；一次 aligned-depth freshness 告警后自动恢复。诊断节点原先在同一日志调用点切换 INFO/WARN 会触发 rclpy 异常，本轮仅修正日志调用位置，未改变算法或相机数据。
- **Build/test:** `colcon build --symlink-install` 成功；4 tests passed，0 failed/errors/skipped。完整 `camera_test.launch.py` 运行超过 30 秒并干净退出。
- **Issue:** wrapper 警告 hardware-clock timestamp 可能周期性复位；图像订阅吞吐和跨传感器时间同步需后续专项验证。
- **Safety boundary:** 没有目标识别、三维点发布、外参、map 转换、导航接入或机器人运动命令。

## 2026-09-28 — Live RGB and Aligned-depth Probe

- **Implementation:** 新增 `depth_viewer`，通过 OpenCV 显示 RGB 与归一化伪彩 aligned depth；鼠标左键在 RGB 上选择像素，并对 5×5 ROI 过滤零值/无效值后输出有效数量和米制 median。
- **Inputs:** 实际 topic `/camera/camera/color/image_raw`、`/camera/camera/aligned_depth_to_color/image_raw` 和 `/camera/camera/color/camera_info`；未使用 raw depth 做像素查询。
- **CameraInfo:** 节点实机收到并打印 color CameraInfo，frame、640×480 尺寸和内参与原始消息一致；当前不执行 XYZ 反投影。
- **Display evidence:** 实机 launch 成功创建 `D435 RGB` 和 `D435 Aligned Depth` 两个 640×480 OpenCV 窗口，节点持续收到相机消息并干净退出。
- **ROI tests:** 7 个 package tests 全部通过；覆盖 `16UC1` 毫米到米转换、零值过滤、NaN/Inf/负值过滤、5×5 边缘裁剪和全无效 ROI。
- **Manual near/far click:** `NOT VERIFIED`。当前自动化终端窗口覆盖图形桌面，合成鼠标事件未到达 OpenCV callback；不能据此虚构近处/远处人工点击观测。需用户在可交互桌面执行 launch 后手动点击两个距离不同的目标补录数值。
- **Safety boundary:** 仅订阅相机数据和显示窗口；没有修改或启动任何 GO2 导航/控制代码。

## 2026-09-28 — Blue Top-surface Center

- **Implementation:** 独立 `blue_surface_center` 节点订阅 RGB、aligned depth 和 color CameraInfo。最大蓝色连通区域经腐蚀后反投影为局部相机点云，顺序 RANSAC 最多提取多个平面。
- **Selection rule:** 不按最大平面直接选择。默认以相机向下俯仰45°推导 optical-frame 水平面法向，按无符号法向夹角选择候选；俯仰角、最大法向误差、最少内点和最小内点比例均为参数。未使用已知物体边长。
- **Invalid behavior:** 启动等待数据以及候选法向误差约34–88°时实测输出 `INVALID`；该状态不发布坐标。日志按2秒节流。
- **Valid sample:** 实机随后从 `/science/blue_surface_center` 收到 `geometry_msgs/msg/PointStamped`：frame `camera_color_optical_frame`，示例中心约 `X=-0.129 m, Y=0.206 m, Z=0.836 m`。这是单次功能样本，不代表精度标定或跨场景稳定性结论。
- **Debug:** X11 实测创建 `Blue Surface Center` 窗口；代码覆盖蓝色轮廓、选中平面像素、中心十字、XYZ、有效深度数量与 VALID/INVALID 状态。自动化终端覆盖桌面，最终覆盖层未做独立人工视觉签收。
- **Tests:** `colcon build --symlink-install` 成功；12 tests passed，0 failures/errors/skips。合成测试覆盖实际内参反投影、无效深度过滤、多平面提取、方向先验优先于最大平面、错误法向拒绝和稳健 median 中心。
- **Runtime issue:** 最终独立 launch 复测时现场已有另一个 RealSense wrapper 占用设备，重复 wrapper 报 `VIDIOC_S_FMT: Device or resource busy`；此前连接已有相机 topics 的节点级实测成功。运行时必须只保留一个 camera wrapper。
- **Boundary:** 输出只在 CameraInfo 给出的相机 optical frame；没有 TF 到 GO2/map、导航接入或运动命令。

## 2026-09-28 — Blue Mask and 2D-prior Stabilization

- **Change:** HSV 二值结果先执行参数化 morphology close + open，再保留最大连通区域；不再对完整目标 mask 连续腐蚀两次。
- **Top prior:** 从最终目标 bounding box 上部 `top_region_ratio=0.55` 取候选区域，再以3×3 kernel腐蚀一次去边缘。只反投影该ROI内有效 aligned-depth 点，并只拟合一个主 RANSAC 平面。45°安装先验仅作拟合后法向检查。
- **Debug:** 同一窗口的2×2视图显示标注RGB、原始HSV mask、最终目标mask、上表面ROI/RANSAC内点；叠加 center、XYZ、`mask_area`、`valid_depth_ratio`、`inlier_ratio` 和 `normal_error`。
- **Metrics definition:** invalid frame 为已取得新depth但未发布有效中心的处理帧；loss event 为上一处理帧VALID、当前处理帧invalid的状态转换，启动等待不计入。
- **Continuous result:** 墙钟约30秒，其中从首个可处理帧起连续统计28.0秒：204 frames，86 VALID，118 invalid，11 loss events，VALID ratio `0.422`。有效中心多数时间约在 `Z=0.82–0.89 m`；过程中目标/视场明显移动，末段X从负值变为约 `+0.12 m`，因此该比例不能解释为固定场景精度。
- **Interpretation:** 相比前一轮约7%的早期观测，识别率提高，但当前42.2%仍不能称为稳定。拒绝主要来自平面法向超过35°阈值；未通过放宽到不可靠角度来伪造VALID。
- **Build/tests:** `colcon build --symlink-install` 成功；13 tests passed，0 failures/errors/skips。新增测试覆盖上部比例ROI和单主平面拟合。
- **Boundary:** 未使用已知边长，未接入 TF map、导航或 GO2 控制。

## 2026-09-28 — Blue Surface Invalid-reason Diagnosis

- **Scope:** 保持 `max_normal_error_deg=35.0`、`top_region_ratio=0.55` 及现有检测判定不变，只增加逐帧观测量、INVALID 分类计数和30秒汇总。未接 TF/map、导航或 GO2 控制。
- **Instrumentation:** debug 2×2窗口继续显示 raw HSV mask、最终目标mask、top-region mask；实际送入RANSAC的有效深度像素为洋红色，RANSAC内点为黄色，并显示中心、`(nx, ny, nz)`、mask/ROI/depth/inlier/residual/normal指标。每个新depth处理帧均输出 `FRAME_METRICS`。
- **Runtime setup:** 复用用户已有的单一 `/camera/camera` RealSense发布者，仅独立运行检测节点 `blue_surface_diagnostic_30s`，`validation_duration_sec=30.0`。此前误启动第二个wrapper时观察到 `VIDIOC_S_FMT: Device or resource busy` 和设备反复断连，该次32帧数据作废；随后停止重复launch并在单发布者条件下重跑。
- **30-second result:** 238 frames，0 VALID，238 invalid，VALID ratio `0.000`。INVALID分类：`no_blue=0 (0.000)`、`too_small_mask=0 (0.000)`、`too_few_depth=0 (0.000)`、`no_plane=0 (0.000)`、`low_inlier_ratio=0 (0.000)`、`bad_normal=238 (1.000)`、`timestamp_mismatch=0 (0.000)`。
- **Plane statistics:** `normal_error_deg` count 238，mean `86.660°`，median `86.679°`，max `89.291°`；`inlier_ratio` count 238，mean `0.665`，median `0.666`。典型帧约 `mask_area=14.6k–15.0k`、`top_roi_pixel_count=8.8k`、有效depth约8.2k–8.8k、median residual约2–3 mm。
- **Observed normal:** 拟合法向持续接近相机光学坐标X轴，例如 `(-0.9925, 0.0279, -0.1190)`；在当前45°安装先验下与水平面期望法向相差约85–89°。
- **Diagnosis:** HSV连通区域、top ROI深度覆盖和RANSAC内点比例在本次固定观察中均充足；拒绝不是由蓝色丢失、深度缺失或平面拟合失败造成。证据表明上部55%图像ROI仍主要包含一个稳定的蓝色竖直可见面，单主平面RANSAC因此可靠地拟合了错误语义表面。该结论描述本次场景中的具体失败模式，不证明任何后续策略已经解决问题。
- **Build/tests:** `colcon build --symlink-install --packages-select go2_science_perception` 安装目标已生成；`colcon test` 共14项通过，0 failures/errors。debug窗口的人工视觉内容未单独截图归档，但节点以 `show_debug=true` 完成该30秒运行。

## 2026-09-28 — Complete-mask Multi-plane Recovery

- **Change:** 停用 `top_region_ratio=0.55` 及其腐蚀ROI，不再将上部图像区域作为RANSAC输入。完整 morphology-cleaned 最大蓝色连通区域内的有效 aligned-depth 像素直接反投影为局部点云。
- **Plane extraction:** 顺序RANSAC最多提取4个平面，每次从剩余点中剔除已提取内点。每个处理帧为每个候选输出 `inlier_count`、相对完整点云的 `inlier_ratio`、`median_residual_m`、`normal=(nx,ny,nz)` 和 `normal_error_deg`。
- **Selection:** 先硬过滤 `normal_error_deg <= 35.0`；仅在通过者中按 `inlier_count` 降序、`median_residual_m` 升序选择。因此最大侧面不能绕过法向检查。原有最小表面内点比例检查仍在选择后执行。
- **Debug:** 2×2窗口保留 raw HSV mask 和 target mask；最多4个候选平面使用不同颜色，最终选择面覆盖为黄色，同时显示中心和各候选法向误差。旧 `top_roi_pixel_count` 保留为0以明确ROI已停用，新增 `ransac_input_pixel_count` 表示完整mask输入像素数。
- **Runtime setup:** 复用已有单一 `/camera/camera` wrapper，仅运行节点 `blue_surface_multiplane_30s`，`validation_duration_sec=30.0`、debug开启。收到的有效发布继续使用 `camera_color_optical_frame`。
- **30-second result:** 204 frames，56 VALID，148 invalid，VALID ratio `0.275`，VALID到invalid转换16次。INVALID：`no_blue=1 (0.005)`、`too_small_mask=6 (0.029)`、`too_few_depth=4 (0.020)`、`no_plane=0 (0.000)`、`low_inlier_ratio=19 (0.093)`、`bad_normal=118 (0.578)`、`timestamp_mismatch=0 (0.000)`。
- **Selected/reference statistics:** normal error count 193，mean `49.402°`，median `53.946°`，max `88.798°`；inlier ratio count 193，mean `0.252`，median `0.085`。示例有效帧中，最大侧面候选法向误差 `85.860°`、内点1182；第二候选误差 `11.070°`、内点741，最终正确由第二候选进入发布流程。
- **Interpretation boundary:** 该运行证明多平面提取和“先法向过滤”规则在实机数据上生效，并能避免部分最大侧面误选；27.5% VALID且仍以 `bad_normal` 为主要拒绝原因，不能称为稳定识别或精度验证。现场mask面积变化约482至数千像素，目标/视场并非严格固定，不能与上一轮0%直接作为受控算法优劣比较。
- **Build/tests:** `colcon build --symlink-install --packages-select go2_science_perception` 完成安装；15项package tests通过，0 failures/errors。未接入TF/map、导航或GO2控制。
