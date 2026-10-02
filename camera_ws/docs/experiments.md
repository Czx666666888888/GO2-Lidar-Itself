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

## 2026-09-29 — Distance/scale-adaptive Three-range Validation

- **Scope:** 独立D435蓝色目标检测；单一 `/camera/camera` wrapper，RGB/aligned-depth只读订阅。未启动或修改GO2、TF/map、导航和运动控制。
- **Implementation:** 粗距离定义为完整目标mask内有效aligned-depth的中位数。RANSAC距离门槛分段为 `<1.0 m: 0.008 m`、`1.0–1.5 m: 0.012 m`、`>=1.5 m: 0.018 m`。动态最少内点为 `max(30, ceil(point_count * 0.03))`。
- **Quality gate:** `perception_quality` 同时检查mask面积、有效深度比例、选中平面内点比例和法向误差。默认GOOD门槛为 `800 px / 0.65 / 0.18 / 20°`；MARGINAL门槛为 `200 px / 0.35 / 0.08 / 35°`。仅GOOD发布 `/science/blue_surface_center`；MARGINAL/UNRELIABLE保留2D mask和debug但不发布精确3D。
- **Runtime parameters:** 本表三档实测时launch配置中的 `mount_pitch_deg=45.0` 生效。实测结束后经用户明确要求，源码default和运行YAML已统一为30.0°；因此下表是45°历史实验数据，当前30°配置仍需另行实测，不能把下表结果直接归属于30°。

| 档位 | 粗距离范围 | RANSAC门槛 | 动态min inliers | mask面积 | 帧数 | 质量 | INVALID原因 | 精确3D发布 |
|---|---:|---:|---:|---:|---:|---|---|---:|
| 近 | 0.284–0.286 m | 0.008 m | 222–243 | 8775–8838 px | 49 | UNRELIABLE 49 | bad_normal 49 | 0 |
| 中 | 1.194–1.215 m | 0.012 m | 30 | 728–750 px | 63 | MARGINAL 62; UNRELIABLE 1 | insufficient_quality 62; bad_normal 1 | 0 |
| 远 | 1.795–1.877 m | 0.018 m | 30 | 276–294 px | 67 | MARGINAL 63; UNRELIABLE 4 | insufficient_quality 63; bad_normal 4 | 0 |

- **Near interpretation:** mask和深度点充足，但候选法向误差65.650–78.092°，49帧全部被法向硬门限拒绝。这一场景不能复现此前“近距离好”，说明目标姿态/可见面的语义仍是主要变量。
- **Middle interpretation:** 有效深度覆盖高，选中内点比例中位数0.722、法向误差中位数19.839°；mask面积始终低于GOOD的800 px门槛，因此62帧为MARGINAL并被禁止发布精确3D。
- **Far interpretation:** mask缩小至约276–294 px，但有效深度和RANSAC支持仍高，内点比例中位数1.000、法向误差中位数23.282°；63帧为MARGINAL，4帧法向超过35°为UNRELIABLE，均未发布精确3D。
- **Rejected setup sample:** 首次“中档”摆放实测仅0.969–0.994 m，仍触发0.008 m近档阈值，未计入中档表格。首次近档采样期间目标从约0.90 m移动到约0.28 m，也未作为受控结果使用。
- **Evidence boundary:** 三档结果证明粗距离输出、分段阈值、动态最少内点和质量门控在该次实机数据上生效；所有档位GOOD为0，因此不能宣称精确3D中心已稳定。原始ROS日志位于本机 `/tmp/ros_log_d435_near`、`/tmp/ros_log_d435_mid_retry`、`/tmp/ros_log_d435_far`，不在Git中。

## 2026-09-29 — Coarse Target Locator Four-distance Validation

- **Scope:** 新增并单独运行 `coarse_target_locator`；复用现有蓝色HSV/morphology/最大连通域规则，使用完整mask质心、腐蚀后有效aligned-depth中位数和实际color CameraInfo反投影。不执行RANSAC、平面法向或上表面判断；未修改 `blue_surface_center.py`，未接入GO2、TF/map、导航或运动控制。
- **Output:** `/science/target_coarse_point`，`geometry_msgs/msg/PointStamped`，实测frame均为 `camera_color_optical_frame`。
- **Runtime setup:** 四档均复用单一 `/camera/camera` RealSense wrapper，每档约15秒，debug窗口关闭但逐帧metrics保留。1.5 m首次尝试受隔离环境阻止ROS 2 socket创建而作废；随后在主机ROS环境重跑。期间一次误启动第二wrapper出现 `VIDIOC_S_FMT: Device or resource busy`，该重复实例立即停止，其输出未计入下表。

| 标称距离 | 实测median Z | mask面积 | 质心u范围 | 质心v范围 | 有效深度点 | 帧数/发布/无效 | 发布率 |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.5 m | 0.487–0.488 m | 5914–5949 px | 508.66–509.97 | 210.76–212.43 | 5551–5646 | 37/37/0 | 1.000 |
| 1.0 m | 0.941–0.954 m | 1182–1241 px | 436.44–436.99 | 95.25–95.67 | 1034–1090 | 61/60/1 | 0.984 |
| 1.5 m | 1.467–1.513 m | 550–570 px | 401.24–401.63 | 47.09–47.35 | 463–483 | 38/38/0 | 1.000 |
| 2.0 m | 1.918–1.998 m | 310–328 px | 386.58–386.86 | 23.35–23.63 | 248–263 | 64/64/0 | 1.000 |

- **XYZ ranges:** 0.5 m档 `X=0.150–0.151, Y=-0.027–-0.025 m`；1.0 m档 `X=0.179–0.182, Y=-0.233–-0.230 m`；1.5 m档 `X=0.194–0.200, Y=-0.490–-0.474 m`；2.0 m档 `X=0.208–0.217, Y=-0.724–-0.695 m`。这些横纵坐标随本次目标在画面中的位置变化，不是距离精度指标。
- **Invalid detail:** 1.0 m档有1帧RGB/aligned-depth/CameraInfo兼容性检查失败，随后恢复；其余三个计入档位无无效帧。
- **Evidence boundary:** 该实验确认四个摆放距离下粗定位节点能从实机连续数据发布相机系粗点，并显示目标随距离增加而缩小。它不是测距标定：标称距离未使用外部量具同步记录，未评估真值误差、重复摆放、不同光照、遮挡或视场边缘表现。原始ROS日志位于本机 `/tmp/ros_log_d435_coarse_05`、`/tmp/ros_log_d435_coarse_10`、`/tmp/ros_log_d435_coarse_15_retry`、`/tmp/ros_log_d435_coarse_20`，不在Git中。

## 2026-09-29 — Provisional Camera Extrinsic and Map Stability

- **Scope/safety:** 只启动Point-LIO雷达SLAM、D435 wrapper、`coarse_target_locator`、`camera_target_to_map`、三个SLAM TF桥和RViz。运行节点清单不含FAR、WP5、local planner、path follower或safety gate；Codex未发布任何运动命令，GO2由用户人工移动。
- **Extrinsic under test:** `vehicle -> camera_link` 为 `(0.36, 0.00, 0.12) m`、RPY `(0, +30, 0) deg`；RealSense继续提供 `camera_link -> camera_color_optical_frame`。map链为 `map -> camera_init -> aft_mapped -> sensor -> vehicle -> camera_link -> camera_color_optical_frame`。
- **Interfaces:** `/science/target_coarse_point_map`实测类型为 `geometry_msgs/msg/PointStamped`、唯一发布者为 `camera_target_to_map`；Marker topic `/science/target_coarse_point_map_marker`实测类型为 `visualization_msgs/msg/Marker`，RViz为订阅者。Marker样本frame为 `map`、球体直径0.12 m、lifetime 0.5 s。
- **Discarded trial:** 首轮B位置中用户同时移动了蓝色物体，A/B不满足“目标静止”条件，数据作废，不用于结论。

| 有效档位 | 样本数 | map均值 `(x,y,z)` m | 帧内范围 m |
|---|---:|---|---|
| A（移动GO2前） | 25 | `(1.010748, -0.186983, -0.253594)` | x `1.004856–1.018368`; y `-0.197164–-0.174825`; z `-0.257233–-0.251401` |
| B（仅人工移动GO2后） | 32 | `(1.023916, -0.192327, -0.258100)` | x `1.014261–1.033812`; y `-0.204903–-0.182808`; z `-0.261950–-0.255126` |

- **A/B comparison:** 均值差为 `(dx,dy,dz)=(+0.013168,-0.005344,-0.004506) m`，三维欧氏差约 `0.0149 m`。在本次单次、同一静止目标、人工移动GO2的条件下，map坐标“基本稳定”。
- **Runtime counters:** 整轮粗定位 `frames=2522, published=2347, invalid=175, publish_ratio=0.931`；`camera_target_to_map`最终日志为 `transformed=2347, failed=0`。Point-LIO完成IMU初始化，RViz显示map frame的注册点云与目标Marker。
- **Cleanup note:** Ctrl-C后RealSense和RViz干净退出；现有Point-LIO在DDS publisher析构阶段以 `exit code -11`结束，Python节点/静态TF进程有SIGINT清理trace。该清理问题发生在数据采集之后，不改变A/B样本，但需后续单独处理。
- **Evidence boundary:** 14.9 mm只是一轮功能性A/B结果，不是外参标定精度，也没有验证重复移动、纯旋转、不同距离/方位、SLAM回环或长期漂移。外参状态仍为 `PROVISIONAL / NOT CALIBRATED`，本轮未接FAR/WP5或导航消费端。日志位于 `/tmp/ros_log_d435_map_validation`、`/tmp/ros_log_d435_map_sample_a2_stats`、`/tmp/ros_log_d435_map_sample_b2_stats`，不在Git中。

## 2026-10-02 — Fixed-target Multi-view Map Stability

- **Scope/safety:** 蓝色目标由用户确认全程固定；GO2由用户人工搬动或原地旋转。只启动Point-LIO、D435 wrapper、`coarse_target_locator`、`camera_target_to_map`、静态TF桥和初始RViz显示；未启动FAR、WP5、local planner、path follower、safety gate，未发布运动命令。为降低采集负载，左侧补采前关闭了RViz，SLAM与map基准未重启。
- **Extrinsic/status:** 继续使用 `vehicle -> camera_link = (0.36, 0.00, 0.12) m, RPY=(0,+30,0) deg`。本轮没有修改感知算法、外参、topic、frame或QoS；外参仍为 `PROVISIONAL / NOT CALIBRATED`。
- **Method:** `/science/target_coarse_point_map` 的 `PointStamped` CSV按稳定静止段采集。移动/旋转后的10秒稳定窗口不进入旋转组统计；左侧移动后因TF落后而产生的无输出窗口也不计入样本。下表 `std` 为总体标准差（`ddof=0`），`range` 为 `[min,max]`。每组“总体均值”是该组全部有效样本的合并均值，因此样本较多的段权重更高；`Δxy` 是各段均值减该组总体均值，`|Δxy|` 为二维欧氏距离。

### 平移测试

平移组总体样本数为141，总体均值为 `(x,y,z)=(0.674372,-0.060678,-0.251524) m`。

| 位置 | n | mean `(x,y,z)` m | std `(x,y,z)` m | range x / y / z m | `Δxy` m | `|Δxy|` |
|---|---:|---|---|---|---|---:|
| 正面基准 | 89 | `(0.645808,-0.057110,-0.254509)` | `(0.002769,0.005787,0.001270)` | `[0.638653,0.652708]` / `[-0.071230,-0.042659]` / `[-0.257249,-0.251304]` | `(-0.028564,+0.003568)` | 0.028786 m |
| 左侧平移 | 27 | `(0.710857,-0.053269,-0.242189)` | `(0.005414,0.007738,0.001675)` | `[0.701290,0.720008]` / `[-0.069620,-0.039021]` / `[-0.244754,-0.238626]` | `(+0.036485,+0.007408)` | 0.037230 m |
| 右侧平移 | 25 | `(0.736656,-0.081380,-0.250979)` | `(0.006794,0.015994,0.002060)` | `[0.726235,0.752098]` / `[-0.101475,-0.047947]` / `[-0.256824,-0.246559]` | `(+0.062284,-0.020702)` | 0.065635 m |

- **位置均值两两差:** 正面↔左侧XY为65.2 mm（三维66.3 mm）；正面↔右侧XY为94.0 mm（三维94.1 mm）；左侧↔右侧XY为38.2 mm（三维39.2 mm）。本轮多视角平移结果没有复现上一轮单次A/B的14.9 mm三维差，说明当前暂定外参/map链在这些人工摆位下存在可观的视角相关系统偏差。

### 原地旋转测试

右侧平移段结束时的25个静止样本同时作为旋转组“中间航向”基准；随后用户确认保持机身中心位置不变，分别原地左转和右转。旋转组总体样本数为183，总体均值为 `(x,y,z)=(0.737634,-0.075991,-0.250774) m`。

| 航向 | n | mean `(x,y,z)` m | std `(x,y,z)` m | range x / y / z m | `Δxy` m | `|Δxy|` |
|---|---:|---|---|---|---|---:|
| 中间航向 | 25 | `(0.736656,-0.081380,-0.250979)` | `(0.006794,0.015994,0.002060)` | `[0.726235,0.752098]` / `[-0.101475,-0.047947]` / `[-0.256824,-0.246559]` | `(-0.000977,-0.005389)` | 0.005477 m |
| 左转航向 | 70 | `(0.723137,-0.078191,-0.246308)` | `(0.006163,0.013562,0.001858)` | `[0.710182,0.737217]` / `[-0.109789,-0.054814]` / `[-0.250104,-0.241940]` | `(-0.014497,-0.002200)` | 0.014663 m |
| 右转航向 | 88 | `(0.749443,-0.072710,-0.254268)` | `(0.004904,0.016131,0.002442)` | `[0.734634,0.758944]` / `[-0.103106,-0.034283]` / `[-0.259445,-0.248897]` | `(+0.011809,+0.003281)` | 0.012256 m |

- **航向均值两两差:** 中间↔左转XY为13.9 mm（三维14.7 mm）；中间↔右转XY为15.4 mm（三维15.8 mm）；左转↔右转XY为26.9 mm（三维28.0 mm）。本轮原地旋转的视角相关偏差小于平移组最大偏差，但仍不是零。
- **Runtime evidence:** 启动前D435以USB `8086:0b07`枚举，`enp12s0`为`UP/LOWER_UP`且地址为`192.168.123.99/24`，`/utlidar/cloud`约14.9 Hz。采集期间粗定位最终计数为`frames=6217, published=6165, invalid=52, publish_ratio=0.992`；map转换日志末值为`transformed=2398, failed=3262`。大量失败是按相机原时间戳查询TF时出现future extrapolation，尤其在人工移动后导致map点间歇发布；各表仅统计实际成功发布的map点。这种成功样本筛选和不等样本量是结果限制，不能将表中标准差解释为完整时间序列稳定性。
- **Placement uncertainty:** “正面/左/右”和“中间/左转/右转”来自用户现场人工摆放确认；没有外部量具、定位基准或独立姿态传感器记录实际平移距离、旋转角度及“纯平移/纯旋转”误差。因此分组意图已区分，但几何纯度为 `NOT VERIFIED`。
- **Cleanup:** 采集结束后ROS graph为空，`/science/target_coarse_point_map`不存在；未发现D435、Point-LIO、转换、RViz、FAR、WP5、local planner、path follower或safety gate残留进程。Point-LIO在Ctrl-C析构时再次以`exit code -11`结束，`transform_everything`以SIGINT/KeyboardInterrupt退出；两者发生在数据采集完成后。
- **Evidence boundary:** 本轮证明同一固定目标在五个名义视角下能取得map粗点，并量化了段内离散和段间偏差；结果不构成外参标定、SLAM精度验证或导航可用性证明。原始CSV位于本机`/tmp/d435_multiview_*.csv`，ROS日志位于`/tmp/ros_log_d435_multiview_20261002`，均不在Git中。

## 2026-10-02 — TF Future-extrapolation Diagnosis and Exact-time Retry

- **Scope/safety:** 未修改`coarse_target_locator`或目标检测算法、HSV/depth逻辑和外参。只运行Point-LIO、D435、粗定位、map转换和静态TF桥；RViz在复测前关闭以降低负载。未启动FAR、WP5、local planner、path follower或safety gate，未发布运动命令。
- **Timestamp path:** `coarse_target_locator`复制aligned-depth消息的原始`header.stamp`到PointStamped。`transform_sensors`在首个雷达包到达时计算一次`host_now - raw_lidar_stamp`，随后把同一偏移加到LiDAR和IMU时间戳。Point-LIO的动态`camera_init -> aft_mapped` TF使用处理帧的`lidar_end_time`。`camera_target_to_map`的TF buffer运行wall time并按PointStamped原时间查询。
- **Live clock evidence before fix:** `/laserMapping use_sim_time=True`，其他相关节点为`False`；`/clock`发布者为0且只有`laserMapping`订阅。D435粗点相对主机当前时间实测滞后约0.08–0.16 s，而`/state_estimation`时间戳约超前主机0.055 s。Point-LIO TF header来自传感器帧而不是`node->now()`，所以无`/clock`不是future extrapolation的唯一直接原因，但这是实机launch中的时间源配置不一致。
- **Failure timing evidence:** 修复前多视角日志末计数为`transformed=2398, failed=3262`，成功率`42.3675%`。旧实现每2 s节流留下372条可解析future样本：`point_timestamp - latest_tf_timestamp`最小`0.000229 s`、均值`2.820318 s`、最大`6.590080 s`。失败既包含毫秒级“点刚好比TF新”，也包含人工移动后Point-LIO处理/发布落后约5–6 s；旧同步回调只等待0.20 s，因而直接丢弃这些点。
- **Fix:** Point-LIO XML launch新增`use_sim_time`参数并保持默认`true`；仅`d435_slam_target_validation.launch.py`显式传`false`，实测`/laserMapping use_sim_time=False`且无`/clock`topic。`camera_target_to_map`不再用阻塞0.20 s的一次性查询：首次精确时间查询失败时，为每个点记录`point_timestamp`、`latest_tf_timestamp`和`time_delta_sec=point-latest_tf`，再按原始时间戳进入最多128点、最长8 s、20 ms周期的有界重试。没有任何路径把失败点改成查询latest TF；只有精确时间戳可用才发布，超时/溢出/关闭未完成才计入最终failed。
- **Build/tests:** `colcon build --symlink-install --packages-select go2_science_perception`成功；package共31 tests通过，0 failures/errors/skips。新增测试覆盖纳秒时间转换和正/负`point-latest_tf`差值语义。
- **Fixed-target multi-view repeat:** 用户确认目标固定，依次人工完成正面、左侧、右侧、原地左转和原地右转。每个稳定段各采15 s；两个独立CLI订阅器观察到的输入/output行数分别为`82/83`、`61/61`、`94/86`、`66/68`、`94/94`。这些窗口计数受订阅器启动/退出边界、接收吞吐和跨窗口pending完成影响，不作为节点丢包率；最终比例只采用节点内部守恒计数。
- **Final counters:** 整轮`input=3579, transformed=3579, failed=0, initial_miss=1, deferred_success=1, success_ratio=1.000000`，即`transformed/failed=3579/0`、成功率100%。唯一miss发生在启动瞬间，日志完整记录`point_timestamp=1790931533.881758789`，当时`camera_color_optical_frame`尚未进入buffer，因此`latest_tf_timestamp=unavailable, time_delta_sec=nan`；该点随后以同一原始时间戳重试成功。移动和旋转过程中没有新增miss或drop，因此本轮没有数值型失败delta可记录。
- **Interpretation:** 这次结果证明在本次约10.8分钟、五个名义视角的运行中，wall-time一致化加精确时间戳有界重试消除了最终转换失败，且没有用latest TF掩盖问题。它不能单独区分改善来自wall-time配置、较低RViz负载还是运行时Point-LIO未再次出现上一轮同等级5–6 s积压；8 s以上的TF延迟仍会明确失败。也不证明外参、SLAM精度或导航可用。
- **Cleanup/evidence:** 停止后ROS graph为空且目标map topic不存在；未发现D435、Point-LIO、转换、FAR/WP5或控制残留进程。Point-LIO和一个static TF进程在Ctrl-C析构时以`exit code -11`结束，不影响已完成计数。原始CSV位于`/tmp/d435_tf_fix_*.csv`；修复前日志为`/tmp/ros_log_d435_multiview_20261002`，修复后日志为`/tmp/ros_log_d435_tf_fix_20261002_run2`，均不在Git中。
