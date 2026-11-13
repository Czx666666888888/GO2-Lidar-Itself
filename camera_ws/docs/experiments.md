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

## 2026-10-02 — Post-TF-fix Quick Map Stability Retest

- **Scope/safety:** 用户确认蓝色目标全程固定，GO2由用户人工完成正面、左侧平移、右侧平移、原地左转和原地右转摆位。只启动Point-LIO、D435、`coarse_target_locator`、`camera_target_to_map`和TF桥；采样前关闭RViz。未启动FAR、WP5、local planner、path follower或safety gate，未发布运动命令。本轮没有修改代码逻辑、感知算法或外参。
- **Configuration:** `vehicle -> camera_link`保持`x=0.36, y=0, z=0.12 m, roll=0, pitch=30 deg, yaw=0`，状态仍为`PROVISIONAL / NOT CALIBRATED`。实测`/laserMapping use_sim_time=False`；`/science/target_coarse_point_map`类型为`geometry_msgs/msg/PointStamped`且唯一发布者为`camera_target_to_map`。
- **Method:** 每个稳定位置从`/science/target_coarse_point_map`采集约13.5–13.9 s CSV。下表`std`使用总体标准差（`ddof=0`）；`x/y range`同时给出`[min,max]`及跨度。右侧平移段作为原地旋转组的中间航向基准。

| 分组 | n | mean `(x,y,z)` m | std `(x,y,z)` m | x range m（跨度） | y range m（跨度） |
|---|---:|---|---|---|---|
| 正面 | 107 | `(0.936451, 0.331417, -0.208984)` | `(0.004173, 0.005373, 0.001594)` | `[0.924585,0.946677]`（0.022092） | `[0.316224,0.344194]`（0.027970） |
| 左侧平移 | 94 | `(0.923408, 0.333871, -0.203792)` | `(0.004856, 0.006137, 0.001991)` | `[0.912386,0.936110]`（0.023724） | `[0.315571,0.350754]`（0.035183） |
| 右侧平移 / 中间航向 | 101 | `(0.948073, 0.330087, -0.212247)` | `(0.004183, 0.006358, 0.002065)` | `[0.936837,0.960043]`（0.023206） | `[0.316627,0.345139]`（0.028511） |
| 原地左转 | 92 | `(0.947752, 0.316190, -0.205562)` | `(0.005048, 0.004838, 0.001793)` | `[0.934221,0.957758]`（0.023537） | `[0.305532,0.328773]`（0.023241） |
| 原地右转 | 94 | `(0.951451, 0.341693, -0.215285)` | `(0.004936, 0.005351, 0.001993)` | `[0.936993,0.962441]`（0.025448） | `[0.325300,0.351976]`（0.026676） |

- **平移组均值XY距离:** 正面↔左侧`13.27 mm`，正面↔右侧`11.70 mm`，左侧↔右侧`24.95 mm`；最大值为`24.95 mm`（左侧↔右侧）。
- **旋转组均值XY距离:** 中间航向↔左转`13.90 mm`，中间航向↔右转`12.09 mm`，左转↔右转`25.77 mm`；最大值为`25.77 mm`（左转↔右转）。
- **Map transform counters:** 节点最终守恒计数为`input=3817, transformed=3816, failed=1, initial_miss=1, deferred_success=0`，即`transformed/failed=3816/1`、成功率`99.9738%`。唯一miss发生在启动阶段：`point_timestamp=1790932607.428428467`，当时`latest_tf_timestamp=1790932607.535202503`，`time_delta_sec=point-latest_tf=-0.106774036 s`。该点早于buffer中最早可用TF，精确时间查询等待8 s后仍不可能恢复，最终记录`pending_timeout`；超时时最新TF为`1790932615.586625099`、delta为`-8.158196632 s`。没有改查latest TF或以latest TF发布该点。
- **Interpretation:** 本轮平移与旋转的最大组间均值XY差均约`2.5–2.6 cm`，因此在这次固定目标、五个名义视角的快速复测中，**没有观察到约5–10 cm级多视角map偏差**。但这只是一次人工摆位复测，实际平移距离、旋转角度与几何纯度没有外部量具或独立定位真值，不能据此宣称外参已标定或长期稳定性已验证。
- **Cleanup/evidence:** 停止后ROS graph为空，未发现D435、Point-LIO、粗定位、map转换或TF桥残留进程。Point-LIO在Ctrl-C析构时以`exit code -11`结束，`transform_everything`以SIGINT/KeyboardInterrupt退出；均发生在采集和最终计数之后。原始CSV位于`/tmp/d435_quick_retest_{front,left,right,rot_left,rot_right}.csv`，ROS日志位于`/tmp/ros_log_d435_quick_retest_20261002`，均不在Git中。

## 2026-10-02 — Multi-blue-target and Blue-wall Static Validation

- **Scope/safety:** 只启动D435 wrapper和`coarse_target_locator`，未启动Point-LIO、`camera_target_to_map`、FAR、WP5、local planner、path follower或safety gate，未发布运动命令。保留既有HSV候选与`/science/target_coarse_point`接口；没有修改`camera_target_to_map`。
- **Implementation:** 全部蓝色aligned-depth像素反投影到3D；RANSAC主平面仅在支持点数、蓝色点支持比例、图像覆盖范围及相对重力方向的竖直误差同时达标时判为蓝墙。墙内点剔除后，剩余有效深度像素按2D连通组件和`0.08 m`深度间隔聚类。同一2D组件内占比小于`0.15`的深度层作为边界/深度噪声丢弃；独立组件仍允许不同像素面积，不使用固定方块物理尺寸。每个目标分别记录centroid、median depth、camera XYZ，并在同一原始深度时间戳下连续发布。
- **Debug:** annotated画面以红色显示wall/rejected，以循环颜色显示valid target components，并对各目标绘制centroid十字；四宫格同时显示原HSV mask、wall mask和remaining/target masks。
- **Synthetic tests:** 6项通过，包括“仅蓝墙→0目标”、“蓝墙+1方块→1目标”、“蓝墙+不同大小方块→全部保留”及同一颜色连通区按明显深度间隔拆分。源树全量package tests为`32 passed, 2 warnings`；`python3 setup.py build --build-base /tmp/go2_science_pkg_build`成功。既有`colcon --symlink-install`在本机setuptools的`symlink_data`阶段持续停滞，已中止，故本轮colcon完整构建标记为`NOT VERIFIED`。

| 静态场景 | 最终观测窗口 | 蓝墙结果 | 目标结果 |
|---|---|---|---|
| 仅蓝墙 | 10 s | 支持度`0.999–1.000`，竖直误差约`3.8–6.3 deg`，rejected | topic CSV `0`行，不发布目标 |
| 蓝墙+1个方块 | 10 s | 支持度约`0.848–0.850`，持续rejected | CSV `38`点；有效帧均`targets_in_frame=1`；median depth约`0.473–0.475 m`，XYZ约`(-0.077,-0.019,0.474) m` |
| 蓝墙+3个不同大小方块 | 10 s重测 | 支持度约`0.860–0.863`，持续rejected | CSV `111`点，约37帧×3；有效窗口持续`targets_in_frame=3`；mask约`1.2k/5.7k/2.4k px`，median depth约`0.628/0.474/0.545 m` |

- **Tuning evidence:** 仅蓝墙首轮因图像覆盖范围`0.116`略低于初始门槛`0.12`而误发布，门槛改为`0.08`后同场景10 s零发布。三目标首轮出现同一组件内约8–11%支持度的异常深度小层，曾造成3–5个输出；加入`min_depth_cluster_ratio=0.15`后同一现场真值重测稳定为3。失败首轮不作为通过证据，但保留在日志中。
- **Known limitation:** 与蓝墙近乎共面且同色的目标可能同时满足墙平面内点条件，无法仅凭HSV+aligned depth可靠分离；本实现会倾向将其随墙剔除。遮挡边缘、深度空洞、很小或低于最小像素/有效深度门槛的目标也可能漏检。本轮只是三种静态摆放，不证明动态、远距离、强反光或导航条件下有效。
- **Cleanup/evidence:** D435和粗定位节点均干净退出，ROS graph无相关残留；原始CSV为`/tmp/d435_blue_wall_{only_retry,one_target,three_targets_retry}.csv`，ROS日志为`/tmp/ros_log_d435_blue_wall_multi_20261002`和`/tmp/ros_log_d435_blue_wall_multi_retry_20261002`，均不在Git中。

## 2026-10-02 — Science Target Manager Offline Verification

- **Scope/safety:** 仅实现和离线验证导航侧目标管理第一阶段；未启动GO2、D435、
  FAR、WP5、local planner、path follower或safety gate，未发布任何运动命令。
- **Source frame evidence:** Point-LIO `publish_odometry()`静态源码设置
  `/state_estimation.header.frame_id="camera_init"`；`base_odom_node`原样复制header到
  `/base_state_estimation`。因此管理器读取运行时header并按原时间戳TF到`map`，不把
  两个frame的数值直接混用。运行时实际frame和TF连续性为`NOT VERIFIED`。
- **Implemented behavior:** 相同时间戳输入批处理；map XY默认0.20 m的一对一关联；
  每轨迹每帧最多计数一次；默认5帧由CANDIDATE转CONFIRMED；最近15次观测median；
  单调稳定ID；最近未访问选择；从目标朝机器人方向0.40 m的standoff点。
- **Interfaces:** confirmed targets使用`MarkerArray`同时显示sphere与`T<ID>`文字；
  selected target和standoff goal各发布`PointStamped`与`MarkerArray`。可选
  `/science/visited_target_id`只更新访问状态。节点没有`/goal_point`publisher。
- **Tests:** package目录执行`/usr/bin/python3 -m pytest test -q`结果为38 passed、
  2个既有工具弃用warning；新增6项覆盖median抗离群、同帧多目标一对一计数、
  确认帧数、稳定ID/访问后重选、近期窗口及0.40 m几何。真机运行结果为
  `NOT VERIFIED`。隔离`colcon build`成功并安装`science_target_manager`入口；完整
  launch的`--show-args`解析成功。节点在受限环境中到达安全边界启动日志后由3秒
  timeout停止；DDS socket因沙箱权限不可用，因此该项只算process-start证据，不算
  ROS graph、topic数据或TF运行验证。

## 2026-10-03 — Science Target Manager RViz Palette Runtime Validation

- **Scope/safety:** 仅修改`science_target_manager`的RViz Marker配色和selected球体
  尺寸；topic、target ID、关联/确认、选择、visited状态、standoff几何及导航接口均未
  修改。实机链只启动Point-LIO、D435、`coarse_target_locator`、
  `camera_target_to_map`、`base_odom_node`、`science_target_manager`和RViz；ROS graph
  中不存在`/goal_point`，未启动FAR、WP5、local planner、path follower或safety
  gate，未发布运动命令。
- **Implementation:** CONFIRMED改为亮青`(0.0,1.0,1.0)`，SELECTED改为亮红
  `(1.0,0.0,0.0)`，STANDOFF改为亮黄`(1.0,1.0,0.0)`，VISITED改为浅灰
  `(0.65,0.65,0.65)`；文字保持白色。普通球体直径保持`0.16 m`，selected由原
  `1.35x`调整为`1.5x`，实测消息直径为`0.24 m`。
- **Build/tests:** 在宿主ROS 2环境执行
  `colcon build --symlink-install --packages-select go2_science_perception`成功；随后
  package tests为`39 tests, 0 errors, 0 failures, 0 skipped`。2条stderr warning为
  既有`SelectableGroups`弃用提示。受限沙箱内首次build曾在setuptools
  `symlink_data`阶段无输出停滞并被中止；宿主重跑成功，未将中止尝试计为通过。
- **Runtime marker evidence:** RViz成功启动并订阅`/science/confirmed_targets`。
  同一实机MarkerArray样本同时包含T4、T5、T6、T9、T10、T11、T13至T18等多个
  confirmed目标；sphere均为亮青、直径`0.16 m`，对应`T<ID>`为白色。selected
  样本为T6，亮红、直径`0.24 m`；standoff样本同为ID 6，亮黄、直径`0.16 m`。
  namespace和ID组合保持分离，未观察到Marker ID互相覆盖或异常闪烁。
- **Visited/reselection:** 通过既有`/science/visited_target_id`一次性发布`data: 6`。
  后续confirmed样本中T6为浅灰`(0.65,0.65,0.65)`，selected从T6切换为T9，T6
  未继续被选中。selected和standoff每次更新均先包含各自namespace的
  `DELETEALL(action=3)`再`ADD(action=0)`，本轮未观察到旧红色/黄色Marker残留。
- **Runtime bounds/known issues:** 约2分钟运行中粗定位为`frames=593`、
  `frames_with_targets=550`、`published_targets=3192`；map转换为`3108/3108`成功、
  `failed=0`。现场蓝色候选较多且目标轨迹ID增长到至少T18，说明当前检测/关联在
  该场景可能累积额外confirmed轨迹；这是本轮配色修改之外的既有算法行为，尚未
  证明每个ID都对应独立真实目标。停止时Point-LIO仍出现既有`exit code -11`，部分
  Python/静态TF节点出现SIGINT清理trace；发生在采样完成后。停止后ROS graph为空，
  未发现D435、Point-LIO、目标节点、RViz、规划器或控制进程残留。运行日志位于
  `/tmp/ros_log_science_marker_validation`，不在Git中。

## 2026-10-03 — Blue-wall Residual Band Rejection

- **Ground truth/safety:** 现场固定为3个真实蓝色科学目标和1面蓝色墙。只做D435粗
  定位以及随后Point-LIO/map/`science_target_manager`只读复核；未修改manager逻辑，
  未启动FAR、WP5、local planner、path follower或safety gate，未发布运动命令。
- **Root cause:** 旧实现以`wall_distance_threshold_m=0.025`完成RANSAC拟合后，只从
  HSV蓝色mask删除同一窄阈值下的wall inlier。墙平面附近但未进入inlier集合的有效
  深度点仍进入连通域/深度聚类，因而能形成墙残片target。没有通过提高
  `min_component_area_px`规避问题。
- **Implementation:** RANSAC拟合、支持率、图像覆盖和竖直法向判定保持不变；墙判定
  成功后，对全部蓝色有效深度点计算到最终wall plane的绝对距离，并以参数
  `wall_rejection_band_m=0.08 m`删除整条带宽。每个剩余cluster再计算wall distance的
  median/min/max；参数`target_min_wall_separation_m=0.10 m`按median执行二次门控。
  每个候选日志现包含area、centroid、median depth、wall distance median/min/max及
  `ACCEPT`或`REJECT reason`。topic、frame、消息类型、目标聚类尺寸门槛及manager均
  未修改。
- **Build/tests:** `colcon build --symlink-install --packages-select
  go2_science_perception`成功；package tests为`41 tests, 0 errors, 0 failures,
  0 skipped`，另有2条既有`SelectableGroups`弃用warning。新增合成测试证明：(1)距墙
  `0.06 m`、不属于`0.025 m` RANSAC inlier的残片会被`0.08 m`墙带删除；(2)距墙
  `0.09 m`、位于墙带之外的cluster被`0.10 m`median门槛拒绝；(3)更远的不同尺寸
  目标仍保留。
- **D435-only live window:** 相机启动并稳定后连续采集35 s，CSV共402点，按原始
  timestamp严格分为134帧，每帧均为3点，无第4个墙残片。三个真实目标wall-distance
  median约为`0.28 / 0.54 / 0.58 m`；最靠墙目标在末段逐帧min约`0.13–0.18 m`，未被
  `0.10 m` median门槛误删。稳定帧中墙RANSAC support约`0.899–0.904`，窄inlier约
  `58.2k–58.5k`，宽带实际删除约`58.4k–58.6k`蓝色点。
- **Startup evidence:** D435-only整轮（包含自动曝光/深度稳定前的启动段）为282帧、
  887次发布，前几帧曾有7–13个候选；因此402点/134帧结论明确限定为随后连续35 s
  稳定窗口，不把启动瞬态隐藏为全程通过。完整链末值为799帧、2310次发布，包含
  启动期和采样器外时段。
- **Manager live window:** 完整只读链的独立35 s稳定窗口收到480个map点，严格为
  160帧×3。新manager进程最终confirmed集合仅为T3、T4、T5，窗口前后未增长；单调
  ID表明启动期创建过T1/T2两个未达到5帧确认的瞬态track，总创建量至多5个，没有
  再增长到十几个ID。该结果来自前端修复，manager没有增加屏蔽规则。
- **Known limits/cleanup:** 当前`0.08/0.10 m`只由这一固定现场验证；真实目标若与墙
  近共面，仍可能被墙带或cluster门槛删除，需要另做受控距离试验。停止时仍出现既有
  Point-LIO `exit code -11`和若干SIGINT清理trace，均发生在数据采集后。停止后ROS
  graph为空，未发现D435、Point-LIO、perception、manager、RViz、规划器或控制进程
  残留。原始CSV为`/tmp/wall_band_008_010_targets.csv`和
  `/tmp/wall_band_manager_map_targets.csv`；ROS日志位于
  `/tmp/ros_log_wall_band_validation_008_010`与
  `/tmp/ros_log_wall_band_manager_validation`，均不在Git中。

## 2026-10-03 — Three Fixed Targets / Moving GO2 Map Stability

- **Ground truth/safety:** 3个真实蓝色目标在整轮保持固定，GO2由用户人工依次完成
  基准、前移、后移、左移、右移、原地左转、原地右转和斜向移动。保持当前蓝墙
  `0.08/0.10 m`过滤、`camera_target_to_map`和`science_target_manager`；未启动FAR、
  WP5、local planner、path follower或safety gate，ROS graph中无`/goal_point`，
  Codex未发布运动命令。
- **Method:** SLAM/map基准全程不重启。初次manager随相机启动曾确认5个ID，因此在
  前端稳定后只重启manager、清空内存track，再开始正式实验；SLAM、D435和map转换
  连续运行。每次人工移动后等待用户确认停稳，再独立采集15 s，不将移动过渡混入
  阶段统计。CSV记录confirmed sphere、selected sphere和standoff sphere的map坐标与
  target ID。下表std为总体标准差；`XY max offset`为单阶段样本相对该阶段XY均值的
  最大欧氏距离。

| stage | ID | n | mean x/y/z m | std x/y/z m | min-max x m | min-max y m | min-max z m | XY max offset m |
|---|---:|---:|---|---|---|---|---|---:|
| baseline | T1 | 319 | 0.489327/0.039089/-0.254629 | 0.000836/0.001489/0.000509 | [0.487885,0.491259] | [0.035315,0.041566] | [-0.255973,-0.253757] | 0.003784 |
| baseline | T2 | 319 | 0.749478/-0.056842/-0.256266 | 0.001278/0.001600/0.000466 | [0.747132,0.752804] | [-0.060906,-0.054552] | [-0.257327,-0.255294] | 0.004064 |
| baseline | T3 | 319 | 0.463543/-0.208235/-0.267881 | 0.000808/0.001449/0.000557 | [0.461953,0.465219] | [-0.212200,-0.206107] | [-0.268725,-0.266591] | 0.003971 |
| forward | T1 | 292 | 0.488525/0.038147/-0.254268 | 0/0/0 | [0.488525,0.488525] | [0.038147,0.038147] | [-0.254268,-0.254268] | 0 |
| forward | T2 | 292 | 0.753798/-0.060456/-0.253008 | 0.001026/0.001603/0.000558 | [0.752400,0.756069] | [-0.064650,-0.057982] | [-0.254118,-0.252047] | 0.004196 |
| forward | T3 | 291 | 0.462382/-0.209039/-0.267388 | 0/0/0 | [0.462382,0.462382] | [-0.209039,-0.209039] | [-0.267388,-0.267388] | 0 |
| backward | T1 | 303 | 0.487493/0.031734/-0.253863 | 0.000962/0.003456/0.000373 | [0.485581,0.490239] | [0.023392,0.036333] | [-0.254511,-0.253174] | 0.008462 |
| backward | T2 | 303 | 0.745725/-0.063186/-0.257920 | 0.001171/0.003800/0.000563 | [0.743421,0.748042] | [-0.072033,-0.057532] | [-0.259410,-0.256583] | 0.009135 |
| backward | T3 | 303 | 0.461189/-0.215391/-0.267668 | 0.000796/0.003498/0.000626 | [0.459354,0.463799] | [-0.223784,-0.210286] | [-0.268777,-0.266633] | 0.008516 |
| left | T1 | 299 | 0.491666/0.040143/-0.249448 | 0.001927/0.002075/0.000241 | [0.488325,0.495334] | [0.035844,0.044657] | [-0.249848,-0.248919] | 0.004652 |
| left | T2 | 299 | 0.749282/-0.055766/-0.251040 | 0.001334/0.002148/0.000715 | [0.746632,0.753045] | [-0.059450,-0.050726] | [-0.252365,-0.250128] | 0.005095 |
| left | T3 | 299 | 0.462801/-0.206814/-0.262691 | 0.001447/0.002125/0.000544 | [0.460833,0.465951] | [-0.211552,-0.202740] | [-0.263596,-0.261529] | 0.004761 |
| right | T1 | 279 | 0.510669/0.029976/-0.251193 | 0.000890/0.002109/0.000479 | [0.508765,0.513328] | [0.024077,0.033805] | [-0.252495,-0.250347] | 0.006244 |
| right | T2 | 279 | 0.768871/-0.070962/-0.251372 | 0.000800/0.002195/0.000251 | [0.767710,0.770829] | [-0.077805,-0.067829] | [-0.252056,-0.250993] | 0.006844 |
| right | T3 | 279 | 0.477389/-0.222817/-0.266164 | 0.001366/0.002267/0.000556 | [0.475153,0.479565] | [-0.229049,-0.218641] | [-0.267436,-0.265465] | 0.006451 |
| rotate_left | T1 | 270 | 0.508226/0.035203/-0.242941 | 0.001005/0.003214/0.000632 | [0.506302,0.510666] | [0.027870,0.041355] | [-0.244823,-0.241604] | 0.007390 |
| rotate_left | T2 | 270 | 0.766377/-0.062733/-0.243741 | 0.001440/0.003609/0.000882 | [0.763440,0.769804] | [-0.070965,-0.056071] | [-0.245177,-0.242077] | 0.008285 |
| rotate_left | T3 | 270 | 0.478642/-0.214539/-0.258736 | 0.001148/0.003092/0.000571 | [0.476254,0.481052] | [-0.222052,-0.208551] | [-0.259626,-0.257373] | 0.007613 |
| rotate_right | T1 | 276 | 0.508371/0.050059/-0.245650 | 0.000688/0.002134/0.000405 | [0.506744,0.510175] | [0.046273,0.054403] | [-0.246466,-0.245030] | 0.004442 |
| rotate_right | T2 | 276 | 0.777609/-0.049690/-0.247059 | 0.001279/0.002203/0.000361 | [0.775822,0.780668] | [-0.054139,-0.044962] | [-0.247657,-0.246076] | 0.004795 |
| rotate_right | T3 | 276 | 0.482985/-0.205873/-0.266821 | 0.001120/0.002176/0.000618 | [0.480826,0.485634] | [-0.209639,-0.201033] | [-0.268139,-0.265670] | 0.004858 |
| diagonal | T1 | 281 | 0.496625/0.050299/-0.245392 | 0.001503/0.002867/0.000388 | [0.492870,0.499543] | [0.044362,0.056857] | [-0.246650,-0.244243] | 0.006791 |
| diagonal | T2 | 281 | 0.758672/-0.047294/-0.249173 | 0.001931/0.002990/0.000589 | [0.753992,0.761474] | [-0.053952,-0.040739] | [-0.250354,-0.248091] | 0.006920 |
| diagonal | T3 | 281 | 0.470064/-0.196100/-0.263003 | 0.001452/0.002814/0.000647 | [0.465697,0.472269] | [-0.202070,-0.189525] | [-0.264131,-0.262049] | 0.006796 |

- **Original-ID stability:** T1/T2/T3在全部8个稳定窗口持续存在。阶段均值的最大两两
  XY差分别为T1 `27.78 mm`（backward↔rotate_right）、T2 `34.62 mm`
  （backward↔rotate_right）、T3 `27.70 mm`（right↔diagonal）；对应阶段均值Z范围
  为`11.69 / 14.18 / 9.14 mm`。单阶段内部最大XY偏移为T2 backward的`9.135 mm`。
  因此原始三条track在map中达到厘米级基本稳定，但约`3.5 cm`的跨视角系统变化不是
  零，属于明确的视角相关偏差。forward窗口中T1/T3的std为0，表示manager重复发布
  未更新的历史median位置，而不是传感器具有零噪声。
- **Duplicate/ID growth:** baseline到right始终只有T1/T2/T3；rotate_left新增T11；
  rotate_right新增T6和T14；diagonal新增T17，最终confirmed集合为
  `T1,T2,T3,T6,T11,T14,T17`。真实目标只有3个，因此至少4条是重复或伪confirmed
  track。同一真实目标并未可靠地保持“唯一ID”，且manager没有轨迹老化/合并，新增
  confirmed会永久保留。本轮按要求只记录该明确问题，没有修改manager逻辑。
- **Selected/standoff:** selected依次为baseline T3、forward T1、backward T3、left
  窗口内T3→T1、right T3、rotate_left T3、rotate_right T14、diagonal T14。
  standoff与当时selected球体的XY距离在全部窗口均为`0.400 m`（数值范围约
  `0.399–0.400 m`），其阶段均值随人工移动发生变化，说明现有几何计算按机器人位置
  更新。但rotate_right开始selected选择了额外track T14；所以几何关系正确，不代表
  selected的目标语义正确。left稳定窗口内发生一次T3/T1切换，也表明最近目标排序在
  距离接近时可能切换。
- **Conclusion:** 若只看原始T1–T3，静止目标map坐标“基本稳定”的上限为本轮最大
  阶段均值XY差`34.62 mm`，且存在视角相关偏差。端到端目标管理结论为不通过：旋转
  和斜移产生额外confirmed ID，最终7个ID对应3个真值，并使selected转向T14。在解决
  前端视角误检/关联半径与track生命周期问题前，不支持接入FAR或运动控制。
- **Runtime/cleanup:** 整轮前端`frames=4406, frames_with_targets=4251,
  published_targets=12174, invalid=155`；map转换`12158/12158`成功、0失败。停止时
  Point-LIO仍出现既有`exit code -11`及Python SIGINT清理trace，发生在采样完成后。
  临时采集器写盘后因其timer内shutdown方式未自行退出，最终按精确PID停止；随后
  ROS graph为空，未发现D435、Point-LIO、perception、manager、RViz、规划器、控制
  或采集进程残留。原始CSV为`/tmp/science_motion_{baseline,forward,backward,left,
  right,rotate_left,rotate_right,diagonal}.csv`，ROS日志位于
  `/tmp/ros_log_science_motion_stability*`，均不在Git中。

## 2026-10-06 — Target Lifecycle / Duplicate-ID Convergence

- **Scope/safety:** 仅修改`science_target_manager`、其参数和单元测试；蓝色检测、蓝墙
  过滤、`coarse_target_locator`与`camera_target_to_map`未改。实机只启动Point-LIO、
  D435、上述只读感知/map链、`base_odom_node`、manager和RViz。运行时node清单不含
  FAR、WP5、local planner、path follower或safety gate，`/goal_point`为Unknown topic；
  Codex未发布运动命令，GO2全部位移由用户人工执行。
- **Implementation:** track增加`first_seen`、`last_seen`、`observation_count`及
  `CANDIDATE / CONFIRMED / STALE`；默认`candidate_timeout_sec=1.0`、
  `confirmed_stale_timeout_sec=2.0`、`remove_timeout_sec=10.0`。STALE不参与selected，
  再次关联可恢复CONFIRMED。confirmed pair必须连续5帧小于`merge_radius=0.10 m`才
  合并，短暂接近会清零计数；保留状态/观测数/首次出现时间更优的ID，合并当前历史
  窗口并重算median。`association_radius`保持0.20 m。日志包含`NEW_TRACK`、
  `ASSOCIATED`、`CONFIRMED`、`STALE`、`REMOVED`、`MERGED old -> kept`及
  `SELECTED_CHANGED`。confirmed sphere与label namespace均先DELETEALL，防止旧marker
  残留。
- **Build/tests:** `colcon build --symlink-install --packages-select
  go2_science_perception`成功。package目录以系统Python和显式`PYTHONPATH`执行
  `/usr/bin/python3 -m pytest test -q`为`48 passed`、2条既有SelectableGroups弃用
  warning。新增测试覆盖candidate删除、confirmed转STALE、STALE不参与selected、
  STALE最终删除、持续近邻合并、短暂近邻不合并、稳定ID保留、合并median、sphere与
  label marker清除及既有0.40 m standoff。首次在`source /opt/ros/humble/setup.bash`
  后直接运行pytest触发环境中的Python `types`模块冲突，未计为测试结果；按项目既有
  系统Python方法重跑通过。
- **Interrupted run:** 首轮forward采样期间用户报告意外关机；检查发现旧进程仍在，
  且随后日志出现约`x=4997, y=15248, z=43866 m`的明显失效SLAM/map坐标。因此该轮
  baseline/forward全部作废。Codex先Ctrl-C停止旧launch并同时确认OS进程与ROS graph
  为空，再从新的Point-LIO/map原点完整重跑，绝不把失效窗口混入下表。
- **Method:** 三个蓝色目标全程固定。新run前端/SLAM/map基准不重启；用户依次人工
  完成baseline、forward、backward、left、right、rotate_left、rotate_right和
  diagonal，每次用户确认停稳后采集15 s。表中位置为该稳定窗口confirmed sphere的
  map均值；每个窗口均恰有3个active confirmed。

| stage | active confirmed IDs | mean XY m（按空间目标：upper / near / lower） | selected | standoff XY |
|---|---|---|---|---:|
| baseline | T3,T4,T5 | (0.6928,0.1939) / (0.4645,-0.1002) / (0.6948,-0.1527) | T4 | 0.400 m |
| forward | T3,T5,T26 | (0.7039,0.1862) / (0.4791,-0.1049) / (0.7058,-0.1610) | T26 | 0.400 m |
| backward | T3,T5,T26 | (0.7245,0.2121) / (0.4871,-0.0872) / (0.7229,-0.1437) | T26 | 0.400 m |
| left | T3,T5,T26 | (0.7178,0.1970) / (0.4834,-0.0956) / (0.7157,-0.1494) | T26 | 0.400 m |
| right | T3,T5,T26 | (0.7392,0.1782) / (0.5036,-0.1179) / (0.7265,-0.1748) | T26 | 0.400 m |
| rotate_left | T3,T5,T26 | (0.7428,0.1536) / (0.5005,-0.1398) / (0.7262,-0.1967) | T26 | 0.400 m |
| rotate_right | T95,T26,T5 | (0.7317,0.2150) / (0.4959,-0.0895) / (0.7266,-0.1451) | T26 | 0.400 m |
| diagonal | T105,T106,T5 | (0.7431,0.2125) / (0.5026,-0.0953) / (0.7385,-0.1509) | T106 | 0.400 m |

- **Acceptance evidence:** 三个空间目标跨阶段均值最大两两XY差分别为upper
  `64.28 mm`、near `54.32 mm`、lower `54.02 mm`，属于厘米级但仍有明确视角偏差，
  不代表外参已标定。所有8个稳定窗口最终ACTIVE CONFIRMED均为3，未永久累积到7；
  selected只引用当窗3个active confirmed之一，旧T4/T3/T26在STALE/删除后未继续被
  选中。实机日志统计106个`NEW_TRACK`、14263次`ASSOCIATED`、21次`CONFIRMED`、
  18次`STALE`、103次`REMOVED`和12次`SELECTED_CHANGED`，说明允许瞬态track且最终
  收敛。本场景没有`MERGED`实机事件；持续合并只由单元测试验证，标记为
  `NOT VERIFIED` for live merge trigger。各窗口standoff均为0.400 m。
- **Runtime/cleanup:** 有效run前端最终`frames=5176, frames_with_targets=4914,
  published_targets=14792, invalid=262`；map转换`14768/14768`成功、0失败。停止后
  manager、前端和map转换干净退出；Point-LIO仍有既有Ctrl-C析构`exit code -11`，
  `transform_everything`/`base_odom_node`仍有既有SIGINT/shutdown trace，均发生在采样
  完成后。最终同时确认ROS node list为空且相关OS进程不存在。CSV位于
  `/tmp/science_lifecycle_restart_{baseline,forward,backward,left,right,rotate_left,
  rotate_right,diagonal}.csv`，日志位于
  `/tmp/ros_log_science_lifecycle_restart_20261006`，均不在Git中。

## 2026-10-08 — Real-time Tracks with Visited Spatial Memory

- **Scope/safety:** 仅修改`science_target_manager`、参数、测试和文档；蓝色检测、蓝墙
  过滤、`coarse_target_locator`及`camera_target_to_map`未改。不接FAR、WP5、local
  planner、path follower、safety gate或`/goal_point`，不发布任何运动命令。
- **Implementation:** 保留`CANDIDATE / CONFIRMED / STALE / REMOVED`短期track生命周期，
  不再以保持persistent ID作为完成目标。新增进程内`visited_positions` map位置集合和
  参数`visited_radius=0.15 m`。`/science/mark_selected_visited` (`std_msgs/Empty`)把
  当前selected的map位置加入集合；小于半径的重复位置合并。selected仅从active
  confirmed且与所有visited位置XY距离均不小于半径的track中选择。旧
  `/science/visited_target_id`保留为兼容入口，但也转换为空间位置记录。
- **RViz/logging:** `/science/confirmed_targets`继续显示全部active confirmed；空间上已
  访问的track改为灰色并标注`visited/ignored`，历史visited点使用
  `science_visited_positions` namespace显示；selected红色只高亮真正未访问目标。
  日志包含`VISITED_ADDED x=... y=...`、`VISITED_MATCH track=... visited_index=...
  distance=...`、`VISITED_DUPLICATE`和`SELECTED_CHANGED`。
- **Offline verification:** package目录执行
  `PYTHONPATH="$PWD:$PYTHONPATH" /usr/bin/python3 -m pytest test -q`为`54 passed`，
  另有2条既有`SelectableGroups`弃用warning。新增测试覆盖：访问A后排除、A换新ID且
  靠近旧位置仍排除、半径边界及半径外目标可选、多个visited位置过滤、visited去重、
  最近未访问目标选择；既有0.40 m standoff测试继续通过。隔离colcon build成功，
  1个package完成。
- **Process-start check:** 从隔离install overlay在宿主启动manager，节点到达
  visualization-only安全边界日志；3 s外部`timeout`结束时出现既有
  `ExternalShutdownException` traceback。随后确认ROS node list为空且无manager OS
  进程。该项只证明进程能加载参数并启动，不证明topic、TF、marker或实机数据链。
- **Three-target live acceptance:** `NOT VERIFIED`。2026-10-08只读前置检查确认D435
  `8086:0b07`及`/dev/video0`至`/dev/video5`存在，但GO2有线接口`enp12s0`为
  `NO-CARRIER`且无IP，ROS graph为空，无法获得UTLiDAR/Point-LIO/map输入。待固定3个
  蓝色目标并保持只读链运行，
  依次人工触发3次`/science/mark_selected_visited`，在人工移动GO2/改变视角且允许track
  ID变化时记录eligible/unvisited数量`3 -> 2 -> 1 -> 0`、每个visited map位置、新旧
  track ID、匹配距离、误过滤及重复selected。验收不要求原始相机检测数量下降；原始
  检测仍应允许持续看到3个目标。

## 2026-11-03 — Three-target Map, Visited Memory and FAR Dry-run Gate

- **Scope/safety:** GO2仅由用户人工移动并确认停稳；Codex未发送运动、武装、Sport或
  safety-gate命令。FAR阶段未启动WP5和safety gate。`pathFollower`只到
  `/cmd_vel_raw`；没有本机节点发布`/api/sport/request`。GO2 DDS中可见的9个bare-DDS
  Sport request端点不是本轮本机启动节点。
- **Stage 1 — PASS with accepted limitation:** 完整TF链为
  `camera_color_optical_frame -> camera_link -> vehicle -> sensor -> aft_mapped ->
  camera_init -> map`，不是只替换frame_id。初始15 s为50/50帧3目标、150个map点、TF
  失败0；后退恢复窗口67/67帧3目标，旋转窗口85/85帧3目标。人工前进时近处目标离开
  视野，55/55帧只剩2目标；后退后以新ID恢复。用户明确接受“导航到最近目标后重新
  识别”的刷新语义。旋转后相对初始XY变化约43.0、39.7、56.6 mm；selected与
  standoff距离约0.402 m。该结果只支持目标可见时的厘米级map标记，不是外参标定。
- **Stage 2 — PASS with false-positive limitation:** 最终干净manager的T1/T2/T3依次访问，
  日志为`VISITED_ADDED`后`SELECTED_CHANGED 2->1->3->None`，eligible完成
  `3 -> 2 -> 1 -> 0`。RViz一帧确认3个track均为灰色`visited/ignored`，并有3个独立
  `science_visited_positions` marker。遮挡原T2直到active marker消失，再露出后生成
  T70；`VISITED_MATCH track=70 visited_index=0 distance=0.028`，selected保持None，证明
  0.15 m空间记忆不依赖track ID。无遮挡恢复后连续12 s无新增confirmed或selected
  切换。遮挡物进入画面时曾造成短暂额外蓝色track/selected切换，必须记录为限制。
- **Runtime incident:** 第一轮Point-LIO的`transform_everything`在运行约9分41秒后
  `exit -11`，导致state/registered cloud停止而原始UTLiDAR仍约15 Hz。仅重启transform
  造成map坐标跳变和大量新track，该数据全部作废；随后完整重启SLAM并重新建立干净
  manager。有效visited结果来自完整重启后的同一map基准。
- **Stage 3 implementation:** 新增`science_goal_bridge`，订阅
  `/science/standoff_goal`，只接受有限的`map` frame点，锁定首个有效点并仅向
  `/goal_point`发布一次，忽略后续selected变化。D435 RViz增加`/path`显示。包目录测试
  `56 passed`（2条既有SelectableGroups warning），独立symlink build成功。
- **Stage 3 runtime — NOT PASSED:** 复用现有
  `system_real_robot_with_route_planner.launch`启动Point-LIO、terrain、FAR、localPlanner、
  pathFollower和graph decoder；定位约14.8 Hz。WP5、安全门、自动武装均未启动。FAR
  RViz Goal Tool会注册第二个`/goal_point` publisher，因此将该RViz停止并改用D435 RViz
  看`/path`。启动污染轮bridge曾发布`map (0.507,0.049,0.021)`，但当时manager同时确认
  多个启动误检，该轮明确作废。清理manager/bridge后，干净轮先确认T1/T2/T3并选择
  T3，随后又确认额外T6且selected从T3切到T6；因此在重新启动bridge前按安全规则停止。
  有效科学目标是否被FAR接收、`/way_point`、`/path`和`/cmd_vel_raw`均为
  `NOT VERIFIED`，没有生成可验收规划路径。
- **Cleanup:** 所有D435、Point-LIO、target、FAR、terrain、localPlanner、pathFollower、
  bridge和RViz进程最终均不存在。停止阶段Point-LIO仍出现既有`exit -11`，localPlanner、
  pathFollower及terrain进程出现`exit -6`；均发生于Ctrl-C清理，需后续单独调查。
  主要日志在`/tmp/science_stage2_*_20261103`、`/tmp/science_far_nav_20261103`、
  `/tmp/science_far_perception_20261103_run2`、`/tmp/science_far_manager_clean_20261103`及
  `/tmp/science_far_upstream_restart_20261103`，不在Git中。

## 2026-11-06 — Independent Single-target Approach and Remote D435 Dry-run

- **Scope/authorization:** 用户要求独立接近，不运行 FAR/WP5/localPlanner/pathFollower。
  新增控制核心、ROS adapter、参数与单独预览 launch；manager 仅增加带原观测时间及
  first_seen 的 JSON track 快照，保留视觉、墙过滤、TF 和 visited。
  默认 preview 不发布 `/cmd_vel_raw`，无 safety gate/武装/本轮 Sport 发布或自主运动。
  用户确认 GO2 和前方一个蓝色物块保持静止，并授权仅进行 dry-run 锁定。
- **Remote camera:** 用户将 D435 接到 GO2 Orin `192.168.123.18`。SSH/SDK确认 ARM64、
  Foxy、USB 3.2 D435、firmware 5.12.7.150、SDK 2.54.2；设备序列不记入仓库。
  独立 `/home/unitree/go2_d435_foxy_ws` 构建官方 wrapper 4.54.1，两个 package 成功。
  初次构建时主机时钟停在 2026-07-09，出现 clock skew warning；用户后来经 PC 一次性
  局域网授时成功设置为当前 UTC。NTP active 但未同步，重启后的时钟保持 NOT VERIFIED。
  默认域 Cyclone 初次退出 245；Fast DDS 能启动但有 bad_alloc 发现噪声。
  独立域 42 Cyclone 限时采集启动正常；最终使用域 0 双方指定 peer，远端禁用 multicast，
  PC 扩展 auto participant 范围至 60，避免启动 9 个只读节点时编号耗尽。
  参数/话题版本差异由单独配置与 remap 解决，不改 stamp/frame。
- **Real samples:** 15 s PC 接收窗口：RGB 223 帧约 14.890 Hz；aligned depth 216 帧约
  14.352 Hz；CameraInfo 223 帧约 14.887 Hz；均为640×480、camera_color_optical_frame。
  RGB消息年龄0.070–0.122 s，aligned depth 0.070–0.122 s，CameraInfo0.061–0.122 s。
  同时收到原生 LiDAR 223条约14.842 Hz，原始 stamp 比PC落后约337.43–337.51 s；
  既有 transform_sensors 源码在首帧测定时间offset并同时用于cloud/IMU，本轮未修改。
  处理后的 `/base_state_estimation` 实际frame=camera_init，审计年龄约 -0.034 s，
  控制器按原 stamp 经TF转换完整pose到map，不是仅换frame_id。
- **Real dry-run FAILED for intended target selection:** 首次人工确认瞬间锁定最近的T124，
  map约(0.867,-0.427)，机身中心约(-0.415,0.103)，初始距离1.381 m；
  该track与前方实物对应的稳定目标位置不符，因此不能认定锁定正确。
  ALIGN预览vx=0，wz约-0.295 rad/s，约1.63 s后视觉超时进入FAULT，vx=wz=0并锁存。
  这只证明该次预览故障归零，不证明真实机器人StopMove或目标接近成功。
  清理manager启动轨迹并增加轨迹至少存活3 s、IDLE候选预览及明确ID确认后，
  实物T1约(1.117,0.066)；更近的右侧伪目标T6、后来T62约(0.877,-0.438)仍会持续确认。
  明确确认T1时，最近候选不同，正确拒绝并保持IDLE零输出。
  因此仅增加轨迹寿命门槛不足以消除误检，**第一阶段仍未通过，禁止进入阶段二**。
  需要先处理现场误检或在用户明确授权的独立范围内调检测参数，再重做选择验收。
- **Localization speed:** 高频相邻pose差分在用户确认静止时出现0.05–0.19 m/s伪速度，
  改为默认0.5 s位置窗口差分后，审计仍有约0.03 m/s静止漂移。不能把里程计速度当成
  外部真值；ETA在无可靠朝目标速度时无物理到达时间意义。无命令速度代替实测速度。
- **Safety audit:** `/cmd_vel_raw`发布者为0；Sport topic上发现9个
  `_CREATED_BY_BARE_DDS_APP_`原生端点，本轮没有本机ROS Sport控制发布者。
  专用gate配置max_speed=0.30、min_speed=0，默认dry_run=true；真实模式还查询验证gate
  参数/唯一可发现本机Sport输出者。gate源码未改。控制器真正停止及进程死亡时物理
  停止效果均 NOT VERIFIED。
- **Evidence paths:** PC `/tmp/science_approach_readonly_slam_20261106.log`、
  `/tmp/science_approach_coarse_20261106.log`、
  `/tmp/science_approach_live_preview_20261106.log`、
  `/tmp/science_approach_stable_preview_20261106.log`、
  `/tmp/science_approach_stable_audit_20261106.json`；
  第一轮逐周期日志 `/tmp/science_approach_1793961006341780169.jsonl`，
  后续 `/tmp/science_approach_1793961230532526466.jsonl`；
  GO2 `/tmp/science_remote_camera_20261106_peer.log`。均为本机临时证据，不在Git。
  复现、状态/参数与门槛见 `science_target_approach.md`。

### Same session continuation — User tuning and explicitly authorized live runs

本段是同日后续证据，更新上文“尚未真实运动/误选未解决”的时间边界，不作删除或改写。

- **User tuning:** 用户自行将 `coarse_target_locator.py` 默认 `s_min=80 -> 110`、
  `min_component_area_px=100 -> 250`，要求重新编译测试。Codex同步运行YAML为110/250，
  未改HSV/morphology/聚类/蓝墙算法。独立build通过。
  前端稳定20 s窗口155/155帧恰好1目标；干净manager的另一个20 s窗口151/151帧1目标，
  449份快照均只有1个ACTIVE CONFIRMED，198份IDLE预览候选均T1。
  参数服务实读110和250。小片/淡色误检在这两个窗口中消失，不代表所有距离/光照均通过。
- **Tuned real-data dry-run:** 用户确认固定目标和GO2静止；锁定T1后APPROACH_VISUAL
  预览最高0.30 m/s，初始机身中心距离1.531 m。暂停本轮检测进程1.2 s并自动恢复，
  约0.461 s后FAULT visual target lost；复位/重新确认后暂停机身里程计节点1.2 s，
  约0.491 s后FAULT localization timeout。两次归零且恢复输入后继续锁存，raw publisher=0。
  这是输入中断注入，不能写成物块实际遮挡或机器人真实故障停车。
  日志 `/tmp/science_approach_live_fault_check_20261106.json`；窗口摘要
  `/tmp/science_approach_tuned_{frontend,manager}_20261106.json`。
- **Stage-order change and authorization:** 用户先选择人工位移dry-run，但记录准备在执行前
  被用户中断；随后要求尽快真机。用户明确确认场地平整无障碍、目标固定、站稳/模式正常、
  急停可用及监护，授权最高0.10 m/s、最多2 s的单次测试；该轮后又明确要求直接执行
  最高0.30 m/s的完整接近程序。Codex再次说明保留0.80 m机身中心停靠距离，不驶入物块。
  未将尚未完成的人工动态dry-run记为已通过。
- **Timestamp preflight rejection:** 首个0.10真机准备轮因时间戳互锁在IDLE进入FAULT，
  **0 Move**。追加6 s只读采样90条base odom，age=-54.745至-46.800 ms，83条超前>50 ms，
  无>400 ms旧数据；源码确认transform_sensors首帧offset及Point-LIO扫描内时间路径。
  当前控制器future_tolerance配置/默认显式设0.10 s，保留其他时间/TF/跳变检查，不换stamp。
- **0.10 short live run:** 控制器总时限1.5 s，独立监督1.7 s停止，gate min_speed=0、
  max_speed/max_yaw_rate=0.10、cmd_timeout=0.30 s、state_timeout=0.50 s、lease开启。
  T1锁定后初始距离1.508 m，约1.516 s进入run timeout并停止，首次FAULT距离1.418 m，
  后续读数1.413 m；16条Move、13条Stop请求（该采样窗口）、raw最大vx=0.10。
  用户随后明确说“刚才已经往前走了一点”，这支持发生前进，不单独证明停车精度。
  `/tmp/science_authorized_short_trial_executed_20261106.json` 和
  `/tmp/science_approach_real_trial_ready_20261106.log`。
- **Full visible-target run — approach and fault-stop, NOT standoff success:** 第一次完整轮
  启动因监督进程漏source camera overlay导致PackageNotFoundError，在武装/Move前退出；
  补齐overlay后实际执行一次。参数max_linear_speed/max_angular_speed=0.30，
  stop_distance=0.80、tolerance=0.03、allow_odom=false、max_run_time=20 s，监督另设20.5 s。
  全轮锁定T1（65份活动/终态快照没有换ID）；初始距离1.434 m；
  最高raw及Sport Move vx均0.294012 m/s（上限约束通过）；采样58条Move、13条Stop。
  确认后5.727 s发生visual target lost，状态FAULT、vx=wz=0，距离0.917 m，
  yaw_error=-0.09481 rad；最后一份状态在6.427 s，距离0.87486 m，目标最后有效map约
  (1.12786,0.03287)，机身约(0.25940,0.13844)。最后读数不是外部测得的最终静止距离。
  未进入standoff reached，**0.80 m定距停车验收未通过**。APPROACH_ODOM未启用。
- **Physical observation:** 用户现场确认“方向正确，已经停住，无明显异常”。据此记录
  本轮方向与视觉丢失后的物理停车PASS（用户观察），不是所有定位/进程故障物理停车通过。
  监督进程随后停止真实控制器、解除武装并停止gate，raw publisher实读0。
  相机、Point-LIO、转换、检测、manager按用户要求保留用于查看；不会自动重新运动。
- **Actual speed unresolved:** 0.5 s机身位姿差分全轮峰值0.49403 m/s，首次FAULT时0.43536 m/s，
  高于0.30命令上限。其来源可能包含Point-LIO抖动/漂移或真实步态速度，尚无外部真值或
  同步Sport速度记录区分，不能认定真实速度<=0.30。下一次运动前应先核验速度估计及制动
  假设；本轮用户没有报告明显冲动，但该观察不能替代定量速度验收。
- **Offline/source verification:** 最终功能回归77 passed，三个DDS测试默认skip；另在隔离域86
  显式跑三个DDS/TF/gate接口测试通过，相关core+ROS共22 passed。77项为当前工作树结果，
  包含任务开始前已存在的bridge未提交修改；该bridge源码及测试未纳入本轮提交。覆盖合成闭环到点锁存、
  实际ROS transport无raw输出、旋转平移TF、坏时间戳、急停、gate最低速度关闭及超时归零、
  gate profile有效/无效互锁。flake8 --jobs=1及pydocstyle通过，package独立构建通过。
  非视觉丢失故障的真机停车、定距停车、正式外参/净空测量、实际速度上限及阶段三均
  **NOT VERIFIED**。
- **Live evidence:** `/tmp/science_authorized_full_run_20261106.json`、
  `/tmp/science_approach_full_20261106.log`、`/tmp/science_gate_full_20261106.log`；
  完整逐周期控制日志路径记录在approach日志启动行。临时数据未提交Git。

## 2026-11-06 — Camera-origin MAP Anchor Hybrid Approach (after 858f75a)

### Prior visual-loss diagnosis

- 来源：上一轮 `/tmp/science_approach_coarse_tuned_20261106.log`、
  `/tmp/science_approach_readonly_slam_20261106.log`、manager日志及完整run JSON。
  旧FAULT发生在1793962682.124附近，但2681.503/2681.628/2681.753及2682.132/2682.253
  仍持续检测到目标。中心约(344,206)至(345,236)，在画面中部；面积874/714 px、有效
  深度727/537点，超过250 px门槛，median depth约0.778/0.725 m。map转换成功且仍关联T1。
- 主要证据支持：一次约0.38 s的检测/map输出间隔，叠加输入本身的时间年龄，触发旧
  0.5 s新鲜度门槛，并非已证明物块出了视野、HSV消失、面积不足、depth失效或ID删除。
  旧轮未录RGB，无法进一步唯一归因于传输、处理调度或RGB/depth配对，不将推断写成事实。
  本轮未重新调整HSV、蓝墙或深度聚类算法，只增加真实检测轮廓/mask/诊断输出。

### Implementation and preflight

- 目标改为MAP空间锚点：确认前最近2 s不同观测的中位数，至少5个样本、最大散布0.08 m；
  锁定后用alpha=0.15平滑修正。新观测必须同时匹配初始锚点0.20 m及当前锚点0.15 m。
  ID删除/合并/刷新不删除锚点；多重近邻歧义或跳变不更新锚点，不切换其他物块。
- `stop_reference=camera_origin`、`camera_frame=camera_link`、XY容差0.10 m，忽略Z。
  通过原始odom时间戳的map->camera_link TF获取相机原点，包含旋转后的安装偏移；
  不把机身中心冒充相机原点，也不硬加0.36 m。legacy body_center仅为兼容接口保留。
- `allow_odom=true`已允许真实模式。视觉间断保持锚点并继续用实时Point-LIO/TF更新距离，
  默认丢失后最多25 s或累计机身里程计路径2.0 m；定位/TF/时间/状态异常仍停车锁存。
  相机越过锚点平面且不在容差内则FAULT，禁止绕回追逐目标。gate仍为唯一Sport输出，
  min_speed=0，最大命令0.30；保留制动上界、加速度、lease、急停和超时。
- 启动发现旧Point-LIO会话已发散：base odom约(-227345,117170)m、年龄0.952 s。
  当时raw publisher=0，预览拒绝启动。用户再次通过局域网对时，重启只读SLAM/TF/manager
  后坐标恢复米量级、odom年龄约-0.060 s，再重新建立锚点。未修改Point-LIO核心算法。
- 用户报告方块10×10×10 mm（用户提供，未以量具复核），确认同一蓝色物块及沿途/终点
  无机身、腿部、相机干涉，确认急停和监护并授权实机。后续明确要求录像、MAP锁定后
  直接执行，不再逐次弹启动确认；成功轮按该明确授权自动执行一次，不自动重复。
- PC录像直接使用实际RGB，叠加原检测器轮廓/中心点及HSV mask、锚点、相机坐标、XY距离、
  速度、ID、状态和原RGB时间戳；ODOM继续录制并显示`TARGET LOST — MAP ODOM CONTROL`。
  `DETECT=VISIBLE/LOST`表示当前检测，controller visual_visible表示map观测新鲜度，两者
  可能不同。每个视频帧和10Hz控制数据保存到同名JSONL，可按RGB stamp/控制stamp对照。
  录像健康丢失会拒绝启动/触发停车，MP4保留停车后画面。

### Trial A — kp_linear=0.5, timeout FAIL

- 完整实际录像：`camera_ws/records/approach_20261106_205502_671674.mp4`，同名JSONL；
  run摘要：`camera_ws/records/run_1793969701623560310.json`。
- 初始camera XY距离1.319 m；锁定ID13。活动采样325 VISUAL、21 ODOM、4 ALIGN。
  最大命令0.30 m/s；最小距离0.24949 m，但未进入0.10 m范围。
  8–15/15–25/25–35 s平均vx约0.126/0.149/0.151 m/s，对应Sport XY速度均值约
  0.00982/0.00616/0.00627 m/s，后半段实际基本不推进。
  35.037 s触发run timeout，首次FAULT距离0.31391 m，最后读数0.34196 m。
- 用户现场报告误差35 cm，与最后MAP距离约34.2 cm相近。停止原因是总时限，不是视觉
  丢失或到点。证据支持低速推进响应不足；未把命令下发当成物理执行，也未只延长时限。
  该MP4完整解码1391/1391帧、640×480、15fps、37,328,529 bytes，记录保留为失败证据。

### Trial B — kp_linear=1.0, camera XY stop PASS in this scene

- 仅针对控制速度将运行YAML的kp_linear从0.5提高到1.0；安全门最低速度抬升仍关闭，
  保留0.30上限、制动限速。总时限15 s，独立监督15.5 s停止；没有加入固定速度计时走法。
- 本轮初始camera XY距离1.54973 m（启动前现场位置已变化），初始稳定中位数锚点
  (1.365783,-0.180021)m。初始锁定阶段相机约(-0.175229,-0.056675)m。
  锚点只平滑修正同一空间目标；终点锚点(1.394041,-0.126856)m，累计65次更新。
  锁定ID全程为13，即使上游刷新仍依空间关联。
- 采样76次APPROACH_VISUAL、2次APPROACH_ODOM、23次STOP。
  最后连续ODOM从约7.627 s、camera XY约0.145 m开始，依实时位姿继续接近；
  7.82724 s进入`STOP / camera XY aligned`，相机(1.311263,-0.122233)m，
  XY误差0.082907 m，vx=wz=0并经gate发送StopMove。
- 最后观测在10.02722 s：相机(1.299799,-0.131168)m，锚点(1.394041,-0.126856)m，
  XY误差0.094341 m。9 s后稳定窗口误差min/median/max约0.08398/0.09976/0.13427 m，
  存在厘米级Point-LIO/位姿抖动，不将最后一帧当成外部真值。
- 用户现场明确报告：“可以这次距离物体10cm停止很好”。记录为本场景水平接近及停车
  PASS（用户粗测约10 cm），与最后MAP约9.4 cm、中位约10.0 cm一致。
  实测左右偏差未单独提供；这不是正式外参标定或重复精度统计。
- 命令速度峰值0.30 m/s；Sport XY反馈峰值0.31597 m/s。反馈与命令不完全相同，不能
  把软件限速写成真实速度严格不超过0.30的外部测量结论。
  Sport原始mode/gait/error值现写入日志；本次error_code=100含义未确认，不擅自切换模式。
- 成功录像绝对路径：
  `/home/czx666/today_full_20260901/today_full/camera_ws/records/approach_20261106_210841_496906.mp4`
  同目录同名`.jsonl`含逐帧索引、原图时间戳、检测、控制及用户实测事件。
  摘要：`camera_ws/records/run_1793970520470079235.json`。
  MP4完整逐帧解码204/204帧、640×480、15fps、3,598,731 bytes；查看了实际ODOM帧及
  停车帧，不是RViz或模拟视频。预览校验录像3713/3713帧也单独保留。
- 结束后控制器和gate退出、raw publisher=0，录像完成封装；没有追加未授权运动。
  相机/SLAM/检测链与独立只读日志终端按用户需求保留。

### Checks and reproduction boundary

- 只运行必要检查：3项新增锚点/相机参考/无视觉闭环测试，加19项原控制核心兼容测试，
  共22 passed（0.02 s）；compileall、单进程flake8、相关pydocstyle和独立package build。
  没有重复全量测试、启动FAR/WP5/localPlanner/pathFollower或修改蓝色识别算法。
- 新增`watch_approach_logs.sh`/`watch_approach_status.py`，独立终端显示每周期XY误差、
  vx/wz、Sport实际速度、里程计速度和状态；先前脚本因set -e/ROS环境加载退出已修复，
  脚本实际输出及终端启动已检查。后续一键入口默认打开该窗口。
- 一键入口`run_map_anchor_experiment.sh --execute`代表操作者明确启动一次实机实验，
  自动录像、等待稳定MAP候选、锁定、通过原gate启动、停车后保存并撤下控制进程，
  不再额外弹确认。默认预览launch仍不发raw速度。该仓库包装入口在实测监督逻辑基础上
  整理，完成语法/--help检查；没有为了验证包装入口而额外运动。
- 当前视频/原始JSONL保存在本机，`camera_ws/records/`已加入gitignore，避免误提交数据；
  源码、参数、实验记录与复现说明提交Git。外参仍PROVISIONAL；不同地面/姿态/目标的
  重复停车精度、低速响应标定及实际速度严格上限仍NOT VERIFIED。

## 2026-11-07 — 移动搜索 → MAP自动锁定 → 自主接近

来源：本轮实机控制JSONL、实际RGB检测MP4及用户现场反馈。分支
`feature/d435-perception-workspace`，基于`4475d3f`，未启动FAR、WP5、localPlanner、
pathFollower，未修改HSV、Point-LIO或坐标变换算法。暗光蓝色识别沿用用户已实测结论，
本轮不重复其专项验收。外参仍`PROVISIONAL / NOT CALIBRATED`。

### 实现与运行条件

- SEARCH直接加入原`science_target_approach`，同一个`/cmd_vel_raw`发布者经原gate输出。
  启动朝向定义MAP直线，默认1.0 m，搜索时限20 s，横向偏离0.20 m停车；路线结束无
  已确认目标则停车。不包含避障，用户确认路线无障碍、起点和急停就绪。
- SEARCH取消3秒track寿命等待：读取manager新增的`latest_observation`原始MAP观测及
  原始`observed_ns`，candidate阶段开始收集，按源时间戳去重；最近1 s至少5次，XY
  最大两点距离严格小于0.10 m时用中位数建锚点。不是对已平滑track位置做散布检验，
  不要求像素静止。未改原始时间戳TF转换。最后整理另剔除SEARCH启动前的在途观测，
  避免首次发现时间早于运动启动；该整理仅完成针对性检查，未为此再次运动。
- 确认周期发送零速度并记录`SEARCH->APPROACH`；随后复用原ALIGN/VISUAL/ODOM控制。
  ID变化后仍按空间关联小幅修正同一锚点，丢视觉后依Point-LIO与camera_link TF闭环。
  相机原点MAP XY≤0.10 m锁存STOP。原定位异常停车、急停、录像健康和命令竞争检查保留。
- 初始SEARCH=0.15 m/s。前三轮后用户明确要求升为0.20 m/s，最终配置/默认均已修改。
  APPROACH命令上限仍0.30 m/s。独立SEARCH参数覆盖文件保留原人工接近配置不变。
- 起初GO2时钟为2026-07-09，PC为2026-11-07。用户完成对时后重启本轮相机/SLAM链，
  清理跨epoch缓存再运动。一次只读launch在创建节点前段错误，开启fault handler重试
  后正常运行；没有改SLAM算法。原始日志`/tmp/science_search_slam_retry.log`。

### 现场迭代（失败记录保留）

| 轮次 | SEARCH命令 | 观测与结局 | 本机视频名（均在camera_ws/records） |
|---|---:|---|---|
| A | 0.15 m/s | 5帧/0.0654 m散布确认；自动接近后15 s超时，最终MAP约0.130 m；用户实测18 cm、目标正确，未通过10 cm | approach_20261107_171651_853661.mp4 |
| B | 0.15 m/s | 调kp_linear=1.5、braking_latency=0.30后，5帧/0.0503 m确认；目标偏航，长期ALIGN，15 s超时，最终MAP约1.726 m | approach_20261107_171937_504184.mp4 |
| C | 0.15 m/s | 调kp_angular=2.0；有间歇视觉/MAP观测但未满足确认窗口，20 s搜索超时停车，未接近 | approach_20261107_172116_341828.mp4 |
| D | 0.20 m/s | 运动中5帧确认、自动接近、到达停车；用户确认“接近10cm完成” | approach_20261107_172248_186678.mp4 |

A末段命令约0.10–0.13 m/s、后5 s Sport XY均值约0.008 m/s，支持低速推进不足；
B在约0.17–0.29 rad/s命令下转向响应不足。只调整SEARCH覆盖配置中的比例增益与
制动延迟，继续原控制算法、命令限幅及gate min_speed=0；没有用最低速度抬升或
放宽停车误差掩盖失败。制动延迟0.30 s与gate cmd_timeout一致。

### D轮详细证据与边界

- run：`camera_ws/records/run_1794043367276655187.json`。
  录像首个RGB有效检测源时间17:22:47.980103（含启动前画面）；
  首次有效MAP观测源时间2026-11-07 **17:22:49.727169 +08:00**；确认/切换
  **17:22:51.570597**，原始源时间至确认1.8434 s。SEARCH命令启动至确认1.5680 s；
  两者不同是早期在途图像的源时间早于启动，最终源码已排除该类预启动观测。
  1秒指最终确认所用窗口，不意味着所有场景在看到后1秒内必能确认。
- 最终确认窗口：5个不同源时间戳的原始MAP点，最大两点XY距离0.08954 m。
  中位数初始锚点(2.466778,0.416516)m；最终锚点(2.428109,0.434983)m，
  67次空间匹配平滑修正。没有因track ID更新另选物块。
- SEARCH命令最高0.20 m/s，确认前MAP路线进度约0.0695 m，Sport XY反馈峰值约
  0.0921 m/s；这说明实际低速响应与命令不同，不宣称实际速度恰好0.20。
  活动日志15 SEARCH、1 LOCK_TARGET、21 ALIGN、61 APPROACH_VISUAL、15 APPROACH_ODOM。
- **17:23:01.370716**、接近开始后9.8002 s首次进入STOP，原因
  `camera XY aligned`，相机原点MAP XY误差**0.092727 m**，vx=wz=0。
  停车后最后读数0.113017 m，有位姿/锚点估计波动。用户随后确认“接近10cm完成”。
  本场景粗验收PASS，不把最后读数改写成≤0.10 m，不作为外部精密真值或重复精度保证。
- 接近命令峰值0.30 m/s，Sport XY反馈峰值0.3503 m/s；实际速度严格≤0.30 m/s
  **NOT VERIFIED**，不能把软件限幅当作真实步态速度上限。
- 完整实际RGB+原检测结果MP4：
  `/home/czx666/today_full_20260901/today_full/camera_ws/records/approach_20261107_172248_186678.mp4`。
  同名`.jsonl`包含原图/检测时间、逐帧状态和控制信息；另有同目录
  `science_approach_1794043368064451998.jsonl`逐周期控制日志。
  D视频完整解码**241/241帧**。A425/425帧、C394/394帧也完成完整解码。
  原始数据本机保留，不提交Git；用户实测反馈已追加到对应run/录像日志。

### 必要检查与未验证项

独立package build通过；两项新增SEARCH针对性测试加原锚点/控制核心检查共24 passed，
单进程flake8（max-line-length=100）、git diff --check通过。没有新增大量离线测试。
路线无目标结束/定位中断停车有针对性软件检查；现场C验证了搜索超时停止，未将其
冒充1.0 m路线终点停车。1.0 m路线终点真机停止、实际严格速度上限、重复精度和其他
光照/地面条件均**NOT VERIFIED**。结束撤下本轮controller/gate/录像节点，保留只读
相机、SLAM、检测链；不自动重启运动。

结束只读ROS审计：`/tmp/science_search_final_audit.json`确认
`cmd_vel_raw_publishers=0`；仅保留相机、Point-LIO、TF、检测/manager，无FAR/WP5/
localPlanner/pathFollower或controller/gate。最后源码整理后独立build及24项检查再次通过，
未追加运动。

## 2026-11-07 — 用户重复轮末段仍检测但停车（故障分析）

用户报告：“刚才这一次我在最后检测到了但是停止了”，本轮按失败调查，不能沿用D轮成功结论。
run为`camera_ws/records/run_1794043913372908704.json`，视频
`approach_20261107_173154_217135.mp4`及同名JSONL；完整解码332/332帧。
控制日志`science_approach_1794043914129292394.jsonl`，感知链日志
`/tmp/science_search_slam_retry.log`。此次仅停止进程、分析与记录，未调整算法/HSV/阈值，未再运动。

- 17:32:04.734（ROS epoch 1794043924.734215）自动锁定T88，5帧散布0.06541 m，
  锚点(3.696783,0.694228)m，相机初始XY距离2.8175 m。
- 锁定约0.2 s后进入APPROACH_ODOM；随后锚点更新计数始终0。
  检测日志17:32:05–07有16份结果但均无accepted目标，T88于17:32:06.351转STALE。
- 后续检测恢复：17:32:11–14窗口22份检测全部有accepted目标；用户观察成立。
  视频第279帧同时显示`DETECT=VISIBLE`与`APPROACH_ODOM`，可见绿色检测轮廓。
  但两者分别表示前端检测和控制器锚点关联，前端重新看到不等于恢复同一锚点视觉更新。
- 后续MAP原始观测约(3.31–3.47,0.69–0.86)m，相对初始锚点约偏离0.23–0.39 m。
  manager确认了新T89/T90。现有锚点更新需同时靠近初始锚点0.20 m、当前锚点0.15 m，
  且必须唯一新鲜confirmed匹配；因此空间门槛足以排除这些新观测。控制日志确认
  `anchor_updates=0`、视觉年龄持续增长；完整逐次拒绝原因未单独记录，不能仅凭ID变化
  断言是ID锁死，现有算法本来允许跨ID空间重关联。
- **直接停车原因**：17:32:13.534（1794043933.534211），接近开始8.800 s，
  `FAULT / odometry continuation limit`。当时失视觉累计里程计路径**2.00336 m**，
  达到`odom_max_distance=2.0`；并未达到25 s视觉失效时限或总接近时限。
  相机XY误差**1.11348 m**，不是到达0.10 m停车。原gate随后收到停止，最后读数约1.071 m。
  累计路径来自里程计增量，含位姿噪声，不能写成外部测量真实走了2.003 m。
- **已确认与待查边界**：前端末段正常检测、MAP转换有成功输出、锚点未更新、累计ODOM
  路径限额停车均有日志证据。MAP空间观测偏移为何达到数十厘米尚不能唯一归因于
  外参/视角、深度或Point-LIO漂移；不要直接改HSV、放宽空间门槛或只抬高2m限额。
  后续应针对空间拒绝记录原始点/track位置/锚点距离及原因后再验证重关联。
- 用户要求清空进程后，停止PC本轮SLAM/TF/检测/manager launch及GO2远端D435驱动；
  两端进程检查未发现这些项目进程或controller/gate残留。保留全部视频/日志，未停止
  GO2原生控制服务。用户已报告停车；清理本身不替代新的物理停车试验。

### 本轮问题定位与3m目标的关系（源码核对）

本轮失败链路为：**视觉短暂中断 → 新MAP观测偏离原锚点 → 重关联未恢复锚点更新
→ 控制器持续处于ODOM → 失视觉累计路径达到2m → 保护停车**。
问题定位在“视觉恢复后的MAP空间重关联/观测一致性”，直接停止条件本身按配置执行。
不能写成“D435一直没有检测”“3m目标被禁止”“ID一变就失效”或“里程计报错”。
其中空间偏移超出门槛有原始点和源码证据；每次track拒绝的具体分支未逐次记录，
故“所有拒绝均由同一门槛引起”及数十厘米偏移的底层来源仍为`NOT VERIFIED`。

- 源码位置：`approach_control.py::observations()`检查confirmed/观测年龄、初始锚点
  距离、当前锚点距离和唯一空间匹配；只有成功更新时才重置`lost_at`和`odom_travel`。
- `ApproachControl.pose()`在失视觉期间累加逐步MAP位置增量；`step()`在视觉超时后
  检查`odom_max_time`或`odom_max_distance`，触发`odometry continuation limit`。
- 2m限制的是**从上次有效视觉校正中断起累计的路径**，不是SEARCH+APPROACH总路程，
  也不是目标初始距离上限。3m目标只要持续有效视觉更新，不会仅因走满2m触发此条件；
  若锁定后一直无法有效更新，则即使还有目标距离也会在累计2m时停止。其他时限仍独立生效。
- 本轮应先诊断为什么恢复观测与原锚点不一致、以及每次重关联拒绝原因，再决定修复；
  本次提交仅保存失败分析与待办，未修改控制源码、HSV或保护参数，也没有进行新实机测试。
  仅文档检查`git diff --check`，不把此前build/test结果当作本轮故障已修复。

## 2026-11-08 — 固定物块MAP误差诊断（进行中）

来源：本轮只读实机JSONL与分析器输出；用户手持遥控器执行运动，不启动APPROACH、
FAR、WP5或gate。原始视频本机`approach_20261108_210056_081993.mp4`（仍在记录）。
旧失败数据的RGB/Depth时差及缺失项详见`map_drift_diagnostics.md`。

正式采集前完成两主机对时。同一SSH连接5次测量RTT为1.32–1.75ms，GO2减PC的
中点估计约-6.1ms（往返边界约-6.92至-5.17ms）；不把SSH建连时间算成时钟误差。
错误时钟预检与重载诊断前的采集不计正式组。正式A/B使用同一SLAM会话。

固定参考MAP XY=(0.9650134234919904,0.15301439430597485)m，为正式A首份原始观测，
只用于内部一致性，不是外部绝对定位真值。物块相机Z约0.885m，与旧失败远距场景不同。

| 测试 | MAP XY最大偏差 | 95%分位偏差 | 主要关联因素 | 判断 |
|---|---:|---:|---|---|
| A静止15s | 0.02511m | 0.02272m | camera Z跨度6mm；Point-LIO XYZ跨度33.3/24.8/8.6mm，yaw跨度1.22° | 本场景静止噪声为厘米级，未复现23–39cm |
| B首轮，覆盖待核实 | 0.05480m | 0.03786m | 记录yaw仅16.50°跨度，末尾未恢复记录起始yaw；存在边缘观测/视觉间断和T3/T4 | 不作为完整左右20–30°返回验收 |
| C | NOT VERIFIED | NOT VERIFIED | 尚未采集 | 待用户遥控 |

A：118份检测均有有效候选、117份MAP观测、223份Point-LIO；X/Y标准差6.50/4.97mm。
RGB/Depth绝对时差P95=67.201ms；115/117观测有100ms以内Point-LIO前后插值支持。
全体MAP观测均唯一对应T1。Point-LIO最大相邻3D步长15.27mm，采样间隔63.3–78.0ms；
不能凭静止定位噪声就认定Point-LIO异常。

B首轮：233份检测、173份含accepted候选、174份MAP观测（边界有在途MAP事件），
440份Point-LIO；相机u跨度329–581px。原始JSONL完整保留，但动作覆盖待用户核实。
最大相邻Point-LIO步长23.99mm、采样间隔63.8–75.0ms。A/B已记录部分未见23–39cm
突跳；尚不能排除运动幅度、距离和观测表面改变后的系统偏差。

A/B的绝对RGB/Depth时差与参考XY误差Pearson相关分别约0.090/-0.055；该短窗口没有
明显线性相关，不是不同步无影响的证明。B既有覆盖边界不得用于最终根因定论。
统计输出：`camera_ws/analysis/map_drift_20261108/{A,B}/`；原始JSONL在records本机保留。

### B补录与C最终记录、身份审计及结论

B首轮用户确认“可能提前转了”，保留原始文件但不作完整旋转对照。之后先启动B2/C
记录再通知用户操作，保持正式A以来同一Point-LIO会话。用户完成B2与C；C用户明确
“只退回了一部分”，不得将末尾位置差当作回环误差。B2实收yaw相对记录起点
-30.77°至+12.52°，末尾-7.63°；不是严格的左右各20–30°并精确回正。C实收最大
XY位移0.53375m，末尾0.14585m，yaw跨度10.46°，不是严格纯平移。上述均是Point-LIO
读数，人工速度/角度/距离没有独立外部真值。

B2/C并非每帧都看见物块：B2为469份检测/418份有候选，C为421/339。转向出画、
边缘/模糊和候选分裂有录像证据。因此三组数据已经采集，但“全程同一物块可见”
和严格受控的B/C运动条件 **NOT VERIFIED**，不以用户完成动作冒充这些条件通过。

| 测试 | MAP XY最大偏差 | 95%分位偏差 | 主要关联因素 | 判断 |
|---|---:|---:|---|---|
| A静止 | 2.51cm | 2.27cm | 定位/测量厘米级波动；RGB-Depth时差P95 67.20ms | 本场景静止内部一致性记录有效 |
| B2物块身份注释后 | 14.00cm | 4.07cm | 视角/边缘/快速yaw变化与模糊、同物体分裂；另有非物块候选 | 不足以唯一归因外参；严格运动/可见性条件未达 |
| C物块候选注释后 | 13.72cm* | 5.96cm | 像素向右下边缘移动、深度/视角改变，返回不完整 | 未完成同物理起点恢复验证；最大点身份有遮挡限制 |

*三组固定参考与上表A一致。C最大点是单候选代理统计，不能宣称身份逐帧完全确认：
该帧(u,v)约(608.6,433.8)，实际RGB所在右下角被原录像mask tile覆盖。继续保留该点，
不为改善统计删除。B2单候选帧也以同一物块连续性为假设，多候选帧进行了视频复核；
注释不是外部真值。逐帧身份选择与依据在`B2_identity.csv`、`C_identity.csv`，
不使用MAP关联半径筛除误差点，不修改在线算法。

B2全部候选混算最大58.15cm/P95 51.75cm，不能称为同一物块漂移。36个多候选帧已
通过源RGB时间匹配查看：线缆/电脑反光区域的额外候选、物块下方/侧面分裂均可见。
B2选择418个物块观测，C选择339个；完整480/342个原始MAP点仍保留，并有未筛选统计。
B2最大两个物块误差点14.00/12.25cm的RGB-Depth header时差均为0；匹配的源RGB可见
运动模糊。这排除了“所有大误差一定出现在67ms header差帧”的说法，不证明曝光同步。

#### 五项误差分解

1. **处理阶段**：偏差存在于`camera_target_to_map`发布的原始MAP观测，在manager
   关联之前。关联拒绝/新建ID是偏移之后的结果，且额外候选也会制造新的ID。
   本轮实际未启动controller，controller ACCEPT/REJECT新字段只完成源码/测试，
   不宣称本轮完成真实控制器重关联验收。
2. **时间**：A/B2/C有效检测绝对RGB-Depth header时差P95分别67.201/67.194/67.193ms，
   中位均为0。仅使用有Point-LIO前后插值支持的物块行，误差与绝对时差的Pearson
   相关约0.079/-0.009/0.065，未见明显线性关系。header相同不等于真实曝光同步，
   本轮没有原生硬件曝光元数据，旧轮也没有完整同步轨迹，不能断言不同步无影响。
3. **Point-LIO**：A/B2/C最大相邻3D位置步长15.27/42.42/57.73mm，没有发现20–39cm
   单步跳变；B2最大yaw导数约1.81rad/s，旋转并不满足严格缓慢控制条件。静止A的
   位姿波动仍存在；这些导数不是实际物理速度，也不能证明定位准确或排除累积误差。
4. **TF/视角**：117/418/339条选定MAP观测的查询时间都等于源测量时间，使用保存的
   平移/四元数重算，最大算术残差<9e-16m，说明变换计算一致；错误外参、错误frame
   假设、位姿误差仍未被排除。B2误差与yaw/u相关约-0.331/-0.344，C约0.447/0.556，
   有视角相关现象，但混杂运动、深度、取点表面及身份，不能认定外参为唯一根因。
5. **重关联**：manager记录每对观测/旧track的XY距离、实际ACCEPT/REJECT和原因，
   CSV/JSONL及第六张子图保留新旧距离。额外候选与物块分裂已证实存在；原失败轮
   T88→T89/T90的逐次原因/完整TF/Point-LIO原始数据已缺失，仍不能重建完整因果链。

**原失败轮23–39cm根因仍Unknown / needs verification**。本轮未复现该幅度的已核实
同一物块偏移，且距离约0.885m，与旧轮约2.685m/图像上缘的观测条件不同。下一步先
解决可观测性：使用无遮挡源RGB/深度和身份注释覆盖旧远距/上缘场景，并增加独立
姿态/位置或外参参考，才能区分表面取点、时间与定位/外参。若另行进入算法修复，
先审查已确认的候选污染/同物块分裂与RGB-Depth配对，不先改Point-LIO或放宽门槛；
目前证据不支持宣称其中任意一项是旧轮唯一根因，本轮不修复这些算法。

#### 数据、图和验证

- A/B2及首轮B录像：`camera_ws/records/approach_20261108_210056_081993.mp4`，
  完整解码5490/5490；C：`approach_20261108_210833_118452.mp4`，874/874。
- 原始事件：`map_drift_A_20261108.jsonl`、`map_drift_B2_20261108.jsonl`、
  `map_drift_C_20261108.jsonl`；错误时钟预检/动作漏录首轮B单独保留，不能混入正式统计。
- `camera_ws/analysis/map_drift_20261108/{A,B2_selected,C_selected}/`各有六子图、
  逐观测CSV、summary和新旧关联距离JSONL；`comparison.json`有TF重算、位姿变化和相关。
- 原录像tile遮挡源RGB后，只补诊断录像并行`*_raw.mp4`，不改检测/控制。
  静止短录像`approach_20261108_211245_917119{,_raw}.mp4`均完整解码84/84，
  raw实帧确认无覆盖。**不能补回此前被遮挡的像素**。保持时间戳对应JSONL。
- 36项现有相关测试通过，独立package构建通过，新增/修改诊断文件flake8和语法检查
  通过；SIGINT recorder已处理重复shutdown，正常收尾。source显式加载最新诊断以避免
  安装旧副本，最终独立build含全部改动；原始视频与大数据仍仅本机保存。

结束ROS graph只读审计：`readonly_graph_audit.json`实收`cmd_vel_raw_publishers=[]`，
无APPROACH/FAR/WP5/规划或gate节点。已停止PC本轮SLAM/TF/检测/manager及所有录像。
清理远端相机时SSH两次返回`kex_exchange_identification: Connection closed by remote host`，
因此远端相机是否退出为**NOT VERIFIED**；它是只读相机驱动，无本轮运动控制进程。

## 2026-11-08 — 5cm立方体SEARCH→STOP_CONFIRM→APPROACH（实现，实测阻塞）

本轮目标明确为5×5×5cm。基于a8fb268，未修改HSV、Point-LIO、关联半径、ODOM上限
或原接近控制计算。只修改SEARCH交接：≤1.8m连续3个源样本触发零速度，Sport与位姿
确认停稳0.20秒后开始收集全新源观测；1秒窗口≥5帧、源跨度≥0.6秒、散布<0.08m，
且唯一匹配触发时空间种子，才自动交接原APPROACH。停稳到采样全程总限2秒，超时
冷却0.75秒恢复原有界SEARCH。细节、反馈阈值和证据字段见science_target_approach.md。

验证：6个SEARCH专项检查加原MAP锚点/控制检查共28 passed；独立package build通过。
不把上述软件检查当作完整实机流程通过。只读GO2预检时USB仅有根集线器，无D435及
/dev/video设备；现有相机驱动持续No RealSense devices were found。没有启动gate或
真实controller，没有发送运动命令。当前完整流程和重复2–3次均为NOT VERIFIED。

| 版本/证据 | 搜索→确认 | 确认散布 | 接近/停车与成功率 |
|---|---|---|---|
| 旧版11/07 D轮 | 移动中直接锁定 | 0.08954m | 单场景用户确认约10cm，重复精度未验证 |
| 旧版11/07重复失败轮 | 移动中锁定T88 | 0.06541m | 锚点未更新、ODOM累计2m保护停车；不能作成功 |
| 新STOP_CONFIRM | 停稳后只用新样本 | 要求<0.08m | NOT VERIFIED：GO2未枚举D435，暂无物理距离和成功率 |

旧版两轮实际物块尺寸未由本轮重新核验，不能仅因新目标为5cm就假定与历史目标相同。
1–1.5秒是停车确认的期望，不是已测结果；完整流程首次成功后才安排重复。

### 当日晚间实机结果与最终6秒版本（本段替代上面的“未发运动命令”当前状态）

USB重插后GO2实收8086:0b07、RGB/Depth和MAP数据，运动前恢复对时并重建只读链。
每轮由唯一controller和原gate执行，独立只读实时日志终端及双视频保留；没有FAR/WP5。
目标按本轮用户说明为5cm。后续用户要求“起步时看不到目标”，并将物块移远；
因此以下不是固定物块MAP位置的严格重复精度试验，不把异质4轮成功率外推。

| 轮次/版本 | 首次有效检测距离 | 请求停车/停稳距离 | 停稳/静止确认耗时 | 样本/散布 | 结果 |
|---|---:|---|---|---|---|
| 1，2秒总限，初始可见 | 1.154m | 1.157/1.152m | 0.800/0.900s | 6帧/0.01627m | 首次STOP MAP XY=0.09597m；用户“已完成，9cm” |
| 2，2秒，物块移远，初始不可见 | 无有效候选 | 无 | 无 | 96份检测无accepted | 1m路线结束停车；用户确认完成 |
| 3，2秒，初始物块未完整入画 | 1.978m提示 | 1.442m/未满足停稳 | 确认总时限2.000s | 未进入有效采样 | 确认超时、冷却恢复搜索、路线结束停车 |
| 4，4秒，人工复位 | 1.607m | 两次请求1.341/1.165m，短暂停稳时1.190/1.152m | 到约3.700/3.300s才满足停稳 | 最多3帧/0.201s跨度，散布0.0083m | 两次确认超时，最终搜索20秒总限停车，未进入APPROACH |

第一轮接近开始至首次STOP约6.900s，视觉丢失1次，原Point-LIO续航后停车。
停车后最后MAP XY=0.11397m，不将首次STOP误差替换成最终误差，也不替代用户9cm实测。
第二轮末段无遮挡录像可见物块，原始HSV mask最多250px，但经清理及有效深度/cluster
门槛后没有有效候选；不是STOP_CONFIRM拒绝该目标。本轮不调HSV或250px面积门槛。
第三轮零速度指令后Sport速度约0.6秒接近零，但位姿窗口速度仍有下降滞后/波动；
用户观察为“有向前的趋势”，不能把它全部归因定位噪声。按用户要求由2秒改为4秒复测。
第四轮已有静止样本散布仅8.3mm，但帧数/源跨度不足，不能描述为“坐标散布太大”。
4秒包括制动、停稳和采样，停稳过晚导致采样窗口被耗尽；重测没有证明重复成功率提高。
4秒超时在10Hz检查中观测到4.100/4.000s，不能宣称物理过程严格在精确4.000s完成。

原始run依次为`run_1794147715951131870.json`、`run_1794147915207444642.json`、
`run_1794148121523671539.json`、`run_1794148350256621251.json`，均在camera_ws/records。
带检测与无遮挡原始RGB两份视频分别完整解码233/233、184/184、203/203、394/394帧。
汇总见`camera_ws/analysis/stop_confirm_20261108/results.json`，原始视频/大日志不提交。
第三轮有向前趋势为用户实测观察；第三/四轮最终物理停稳未获明确单独确认，不冒充已验收，
日志STOP/零速度与实际物理停车证据分开保留。

用户最终要求“把时间延长到6s，然后提交到git等我明天测试”。当前核心默认和SEARCH
参数总确认上限均为6秒，包含制动/停稳/采样，不另设停稳后6秒，不放宽速度、散布、
关联或ODOM界限。新增6秒等待检查及原相关检查共30 passed；flake8、语法和独立build。
**6秒版实机NOT VERIFIED**，今天不再发送运动命令，未声称成功率提升，不安排自动运行。

收工：四轮runner均已完成自己的stop/disarm/子进程清理；PC只读链已停止。远端相机
收工检查两次SSH握手被主机关闭，因此远端D435是否退出NOT VERIFIED；明天先检查
已有相机进程，避免重复启动。6秒独立build完成，安装后core与源码AST一致，安装YAML
实读stop_confirm_timeout=6.0；无6秒真实运动测试。

## 2026-11-09 — 6秒确认、5 m搜索与0.25 m/s实测

来源：本轮控制JSONL、逐帧检测/双录像、只读DDS预检、NetworkManager日志及用户现场回复。
分支基于3f72cda。用户最初要求ODOM路径上限5 m，首轮后明确SEARCH也为5 m；
速度输入2.5 m/s经用户澄清为0.25 m/s。只修改SEARCH配置、入口范围/默认及相关检查，
不改HSV、Point-LIO、关联或安全门。SEARCH默认5 m/60 s/0.25 m/s；STOP_CONFIRM总限
6 s；SEARCH配置ODOM路径5 m、时间25 s；原人工接近配置仍2 m，不影响其默认。
SEARCH距离为本轮起点沿起始航向的投影，不等于目标距离或ODOM失视觉路径。

预检：D435 8086:0b07已枚举。GO2时钟退回2022年，经本地交互sudo对时后才启动链。
每轮运动前只读采样6秒，连续RGB/对齐深度约14–15 Hz、里程计约15 Hz、Sport约300 Hz，
TF可用，raw发布者为空。独立build完成并核对安装配置/控制器启动参数日志。
31项针对性检查通过，含穿过旧距离/时限后在5 m路线结束归零的离线检查；修改Python
flake8及git diff --check通过。不是全仓库build或全量测试结论。

| 轮次 | 配置 | 观测与结果 |
|---|---|---|
| 1 | SEARCH 1 m/0.20，ODOM 5 m，确认6 s | 1.0059 m路线结束；125检测均无有效候选，raw mask最大219px；用户确认停稳，未进入确认 |
| 2 | SEARCH 5 m/0.25，确认6 s | 最大记录进度3.4134 m；定位超时，用户报告切换遥控器并确认停稳；中断轮，不作验收 |
| 3，恢复链路重试 | SEARCH 5 m/0.25，确认6 s | 第二次停车确认成功，进入原APPROACH，camera XY aligned停车，用户确认实测9.5 cm |

第2轮172份检测中8份有前端目标，但控制器没有形成停车触发候选；不能说整轮前端
没有检测。用户切遥控器与软件观测分开记录：网卡enp12s0后续DOWN，NetworkManager
20:32:55记录carrier-changed，DDS有UDP写失败；定位timeout在接近19.06 s搜索时出现。
不据后续网卡事件时间反推最初丢数据原因，也不将网络事件全部归因遥控器切换。
断网收尾中断，未生成runner正常run摘要；已有视频327/327帧完整解码。
`interrupted_search5m_025_20261109_audit.json`明确为原控制JSONL的post-hoc审计，非原run。
旧进程退出后重建SLAM会话，复用远端唯一相机，再预检后启动第3轮。

第3轮首次有效相机检测距离2.2522 m；第一次请求停车距离1.6626 m，停稳约5.900 s，
6.0003 s确认超时（candidate missing or spatially ambiguous），随后冷却恢复SEARCH。
第二次请求停车距离1.5018 m，停稳距离1.4767 m；停稳1.8999 s、静止采样1.5001 s，
总耗时3.4000 s，6份不同源观测、XY散布0.05256 m，通过确认。
搜索最大投影约2.3559 m时已交接目标，未行满5 m。原APPROACH约9.9998 s进入首次STOP，
MAP XY误差0.07587 m；收尾最后0.08805 m。用户回复“确认，9.5cm”，记录实际停稳和
相机到物块水平间距0.095 m，不用内部MAP误差替换用户实测。
期间视觉丢失7次，最大失视觉累计路径约0.2539 m；45次锚点更新，未触发5 m上限。
命令峰值0.30 m/s，Sport反馈XY峰值约0.3108 m/s；不是实际速度严格限幅的外部真值证明。

原始证据均在camera_ws/records，本机保留、不提交大日志/视频：

- 第1轮`run_1794227344304396734.json`、`approach_20261109_202905_232322{,_raw}.mp4`，
  两份均237/237帧完整解码。
- 第2轮`science_approach_1794227546785068846.jsonl`及
  `approach_20261109_203226_883196{,_raw}.mp4`，两份均327/327帧完整解码。
- 第3轮`run_1794227768484749507.json`、`science_approach_1794227769226490345.jsonl`、
  `approach_20261109_203609_327607{,_raw}.mp4`，两份均560/560帧完整解码，closed=true。
  用户实测单列`user_observation_search5m_retry_20261109.json`。
- 三次只读预检JSON及独立构建/tmp/science_5m_install；原断网感知日志
  `/tmp/science_5m_readonly.log`、重试`/tmp/science_search5m_retry_readonly.log`。

结论：6秒预算内第二次确认及接近在单场景获得用户9.5 cm停稳确认。第一次仍超时，
不宣称每次确认成功、重复精度或成功率提升已验收。真实5 m无目标路线终点、真实5 m
ODOM保护边界、外部速度真值与正式外参标定均为NOT VERIFIED。

### 用户两次实测反馈与版本推送

用户收到上述版本的完整启动命令后反馈“这个实测两次都可以”，并要求push当前实验
程序。该反馈作为用户现场观测记录，支持当前版本两次现场可用；不将前述无目标/人工
接管中断轮改写为成功。已审计轮的9.5 cm实测与560帧视频证据保持原记录；所述两次
未在本轮逐一关联run文件或重新解析日志，不将其推断为额外两次新增运行，亦不填写
各轮距离、耗时、散布或成功率。
本次仅补充文档并推送已有实验实现07292cd及本次文档提交，不新增运动实验。

## 2026-11-12 — 稳定 MAP + S1 首次右侧抓取停靠

- 基线：远端 feature/d435-perception-workspace=e15c66ce2beb998ef9c73464b0bff72cbd4e2266；
  实际测试为其上未提交的独立集成工作树，非纯 e15c66c、非视觉伺服分支。
- 用户授权执行；GO2站立由用户确认。先完成对时、停旧自定义遥控入口、部署原S1交付包。
- SDK枚举 D435=213622070598，D455=231622302209；后者与包内绑定相机一致。
- S1首启 J5=-0.126841rad 超出原±0.08rad窗口，程序拒绝；未扩大窗口。
  用户恢复初始位置后第二次进入ZERO_HOLD，busy=false/navigation_allowed=true。
- 执行命令：`bash camera_ws/scripts/run_map_s1_task.sh --execute --search-distance 5.0 --search-timeout 60.0 --timeout 25.0`。
  本轮实际模式dock_only；没有把它描述为一键完整抓取实测。
- 原SEARCH/STOP_CONFIRM/速度与制动参数沿用稳定配置；新增位置0.02m、航向0.05rad、静止0.5s。
- 本地时间00:16:47.82开始SEARCH，00:16:56.11进入STOP_CONFIRM；首次检测相机距离约2.017m，
  停车请求约1.684m，静止时约1.564m。静止确认总4.400s（其中停车3.300s、采样1.100s），
  5个样本，MAP散布0.03255m；第一次确认即成功。
- APPROACH约00:17:00.51开始；使用MAP_GRASP_DOCK，MAP锚点更新34次，拒绝0次。
  目标离开前视相机后沿Point-LIO继续，未接CameraRelativeMatcher。
- 末端TRANSLATE/TURN_TO_POSITION/FINAL_YAW频繁切换，仅1帧SETTLE。
  最近一次MAP位置误差1.60mm时，航向误差仍约33.8°；不是物理真值或到位证明。
- 00:17:25.51接近运行达到25s，FAULT=run timeout。首次FAULT位置误差3.411cm、
  航向28.68°，预测S1目标=(-12.40,+31.22)cm。
  停车清理后的最后记录位置误差2.527cm、航向28.38°，预测S1目标=(-17.91,+29.55)cm。
  目标MAP=(2.826864,0.025709)，停车目标=(2.869540,0.338565,0.055614rad)。
- 控制器/安全门/本轮录像进程已退出；最后指令为0，录像正常关闭637帧。
  物理停止和S1实际XY尚未由用户测量反馈。没有StandDown/grasp/StandUp调用。
- 原始证据在主工作区 camera_ws/records/：
  `run_1794413805038390515.json`、`science_approach_1794413805800481643.jsonl`、
  `1794413805038390515_control.log`、对应gate/recorder.log，
  `approach_20261112_001645_857781.mp4` 和 `_raw.mp4`（不提交视频）。
- 最小修复：进入位置容差后先制动，保持最终航向阶段；位置短暂越界不立即改追点方位，
  航向完成后仍按原位置阈值复核。未调增益、速度、保护时间/距离或完成区域。
  针对性用例覆盖该抖动触发；44项测试通过，flake8/bash语法通过，单包构建成功。
- 修复后实机停靠、实际推荐抓取区、趴下完成判据、完整S1任务、自动起立：NOT VERIFIED。
  未进行自动重复试验，未合入稳定分支。

## 2026-11-12 00:25–00:33 — 实测姿态配置与完整任务入口测试

用户指示“直接填写然后测试”，授权本轮姿态动作和单目标完整任务；随后确认目标已摆好。
测试源码为 fa8e3c3 上的本次SDK环境/姿态配置修正；没有合入视觉伺服分支。

### 已实测的姿态与SDK问题

- `/usr/bin/python3` 使用GO2默认DDS库时，SDK2申请Sport租约发生原生段错误，退出139。
  faulthandler崩溃路径：cyclonedds/pub.py:189 write -> channel.py:144 ->
  client_stub.SendRequest -> lease_client.__Apply，尚未调用StandDown。
- Python绑定0.10.2的编译路径在 `/home/unitree/cyclonedds_ws/install/cyclonedds`；
  实际默认加载 `/usr/local/lib/libddsc.so.0`。任务进程同时指定CYCLONEDDS_HOME及
  LD_LIBRARY_PATH为前者后，租约成功、正常退出；没有重装或覆盖系统DDS/相机/S1环境。
  依据：本机源码/ldd和实际A/B初始化结果；官方SDK依赖说明亦要求0.10.2。
- 00:28左右StandDown实际返回0；最后3秒15个反馈样本高度范围
  [0.0733307153,0.0733730495]m，median=0.0733568147m，mode全为0。
  最大XY速度1.04e-7m/s，最大yaw_rate0.01704rad/s、gyro0.02450rad/s。
  用户现场回复“趴下稳定”，并确认支撑/线束条件。
- 趴下时StopMove返回-1，起立前置检查因此退出，未调用StandUp。
  修正为：趴下前仍须StopMove成功；已确认趴下且静止、S1为ZERO_HOLD时直接StandUp。
- 00:30左右StandUp实际返回0；最后3秒15样本高度范围
  [0.3300811350,0.3300972581]m，median=0.3300901949m，mode全为0。
  原站立采样另约0.31438m；不能仅以mode区分姿态。
- 配置 `science_s1_posture_go2_20261112.json`：down上限0.084m，up下限0.304m，mode均0。
  由实测稳定范围外扩1cm并向外取整至1mm得到，记录原始文件SHA256。
  数值是反馈判据，不是S1基座高度、独立腹部接触传感或精确物理高度真值。

### 完整入口的真实结果

命令：`bash camera_ws/scripts/run_map_s1_task.sh --execute --full --search-distance 5 --search-timeout 60 --timeout 25 --posture-profile camera_ws/src/go2_science_perception/config/science_s1_posture_go2_20261112.json`。

1. 00:31首次启动在SEARCH前即被定位保护拒绝：localization jump or time regression；
   当时位置出现约(-12.27,-52.95)m异常值。未行走、未请求抓取。
   原只读链在此前姿态采样后已失稳，直接因果尚未确定。
   保留日志后只重启本轮SLAM/感知，未改Point-LIO、外参或保护参数。
2. 00:32重启后再启动：SEARCH成功；STOP_CONFIRM一次成功，6样本、spread=0.02024m，
   总2.400s（停车1.400s、静止采样1.000s）。首次发现约2.006m，停车请求1.784m。
3. 进入MAP_GRASP_DOCK；锚点更新58次，拒绝0。最近位置误差0.031615m，
   未进入0.02m位置容差，因而本轮没有进入BRAKE_FOR_FINAL_YAW或FINAL_YAW。
   末端仍在TURN_TO_POSITION与TRANSLATE之间切换，证明仅保持最终航向的前次修复不足。
4. 接近运行25s后FAULT=run timeout；停车清理后的最后位置误差0.099415m，
   航向误差0.150791rad（8.64°），预测S1 XY=(-0.138857,+0.354112)m，区域判定false。
   目标MAP=(1.853580,-0.014789)，停车目标=(1.914928,0.294947,-0.004349rad)。
5. 本轮控制器/安全门/录像进程退出，最终速度指令为0，录像关闭553帧。
   完整任务中的StandDown、grasp、StandUp均未触发；前述姿态动作是独立参数实测。
   物理停车误差、实际抓取、入盒和一键完成：NOT VERIFIED；不自动再试。

原始数据位于主工作区 `camera_ws/records/`：
- `s1_posture_measurement_down_20261112.jsonl`（默认DDS失败）；
  `s1_posture_measurement_down_fixed_20261112.jsonl`（实测趴下）；
  `s1_posture_measurement_up_20261112.jsonl`、`s1_posture_measurement_up_returncode_20261112.jsonl`
  （StopMove前置条件失败）；`s1_posture_measurement_up_fixed_20261112.jsonl`（实测起立）。
- `run_1794414693037744519.json`、`science_approach_1794414693816367225.jsonl`：启动定位失败。
- `run_1794414768992776436.json`、`science_approach_1794414769750593837.jsonl`：真实停靠超时。
- `approach_20261112_003249_839125.mp4`及`_raw.mp4`：553帧录像；
  `s1_slam_restart_20261112.log`：原参数重启记录。

静态/离线：44项相关回归通过，flake8及diff检查通过；实际配置加载和显式CLI覆盖通过。
改动仅直接执行的协调脚本与JSON，无全仓库构建；完整抓取协调仍未实机验收。
下一步应解决非横移末端停车位置收敛，不扩大完成区、速度或超时来宣称成功。

## 2026-11-12 — 三自由度MAP抓取停靠（仅代码与针对性离线验证）

来源：开始前git fetch，集成分支本地/远端HEAD均为
`4e27f6123f6cd30fecf7d1a9dfa7783d1a0dd821`。用户明确当前无实机条件；
本轮未SSH连接GO2、未启动相机/SLAM/ROS节点或机械臂，没有新的运动/抓取实测结果。
历史两次vx/wz停靠超时仍保留，不能作为新控制的验收证据。

实现：
- 取消TURN_TO_POSITION/TRANSLATE追点切换，按R(-yaw)将MAP位置误差变换为机身ex/ey。
  vx_raw=kp_linear*ex，vy_raw=kp_linear*ey，wz=kp_angular*wrap(yaw_goal-yaw)，
  Kx=Ky沿用原纵向增益，合成平面速度≤原max_linear_speed，角速度原上限不变。
- 沿用制动距离/延迟/实际速度逻辑，扩展为二维向量加减速限幅；接近目标平滑降低vx和vy。
  在MAP中保存历史平面速度，避免机身旋转时错误复用历史速度轴。
- 完成条件为位置≤4cm、航向≤10°、原静止反馈持续0.5s。
  predicted_target_s1_xy和推荐矩形判定继续记录；矩形不再硬性拒绝停车。
  到位只证明导航容差，原S1/D455仍须判断目标可观测和可达。
- core的max_run_time与runner固定总时限只在抓取停靠模式取消；普通MAP和SEARCH不变。
  S1 runner旧--timeout参数仅兼容接收并在计划中注明忽略，新手册命令移除此参数。
- 原ODOM时间/距离预算不改（本搜索配置25s/5m）；定位/TF过期、反馈异常、控制器竞争、
  原安全门、人工/stop、录制心跳等保护保留。runner增加1s控制状态心跳检查。
- 新增唯一卡滞窗口参数dock_progress_timeout=10s；归一化位姿误差在窗口内累计改善不足0.25
  则FAULT且vx/vy/wz全零。该改善量单轴对应1cm或2.5°，微小往复不能续期。
  这是无进展窗口，不是总任务时限；接近一直改善且视觉有效时可超过25s。
- 唯一出口仍为science_target_approach -> /cmd_vel_raw -> 原go2_safety_gate -> /api/sport/request。
  安全门源码已支持linear.y与Move JSON y。vy正值按现有遥控/vehicle约定为左侧；
  Sport反馈XY用于速度范数，闭环方向来自MAP->vehicle。真实横移符号与速度跟随NOT VERIFIED。
- S1抓取/姿态协调文件、实测高度配置(0.084/0.304m)、HSV、Point-LIO、外参和安全门源码未改。

验证结果：
- 66项定向测试通过：左右vy、90°变换、前后/横向/转向同时输出、平面速度/加减速、
  4cm/10°边界和0.5s停稳、推荐区非硬门槛、STOP/FAULT全零、普通MAP vy=0，
  以及超过25s仍进展/10s卡滞/独立ODOM保护/定位过期等边界。
- 以假时钟和假发布器检查runner：抓取停靠允许超过原SEARCH+25s总期限，普通MAP仍保持
  85.5s组合watchdog；未创建ROS节点、未发布DDS或机器人消息。
- ROS消息适配方法直接用假对象调用，检查TwistStamped.linear.y及故障清零，不初始化ROS。
- 40个普通MAP/SEARCH输入快照与修改前HEAD结果逐字段相同，仅新增显式vy=0。
- 原S1任务完成/FAULT/旧任务结果判断回归通过；姿态配置和动作链文件未改。
- flake8、手册bash语法、完整离线计划/姿态配置加载通过；仅go2_science_perception单包构建通过。
  没有全仓构建、批量仿真或真实launch。

下一次最值得实测的三项：
1. 正/负vy与左右实际位移符号，以及指令和Sport/Point-LIO速度的跟随与合成速度峰值。
2. 停靠位置/航向误差是否共同收敛并保持0.5s，记录完成耗时、超调及S1实际XY。
3. 视觉离开视野后的实际路径、ODOM剩余预算与卡滞判定；明确区分ODOM保护和无进展停止。

三自由度横移控制已实现，但尚未实机验证。若原ODOM预算阻挡合理末段，需用上述实测证据
单独讨论定位可信度策略，本轮没有悄悄扩大或删除该预算。

## 2026-11-12 — LiDAR/IMU真实时间链诊断（不运动，源时钟仍阻塞）

开发起点：实际fetch的集成分支a8f7c667874b3416bb9af5a01b2ad1d0560f6442。
本节随本次代码提交一起保存；最新实现commit见本节所在提交的`git log -1`，
原始metadata里的a8f7c667是采集时基线，不冒充后续实现提交。
未更改稳定MAP或视觉伺服分支；主目录无关bridge/训练工作保留。

### 实测环境和采集方式

PC有线enp12s0=192.168.123.99，伴随主机.18可SSH，S1保持服务保留。
开始检查时PC没有Point-LIO，诊断才启动仅传感器转换、Point-LIO、base_odom与必要静态TF。
已存在SLAM时脚本只订阅并保留状态，不替换进程；不会启动相机、遥控、速度控制或姿态任务。
结束只关闭本轮创建的只读进程。全程没有发布运动指令，未进行仿真或批量单元测试。

真实读回：laserMapping use_sim_time=false；transform_everything=false；/clock发布者0；
time_lag_imu_to_lidar=0；publish_odometry_without_downsample=false；cut_frame/con_frame=false；
timestamp_unit=0（time为秒）。PC systemd-timesyncd报告已同步，但timesync-status同时显示
公网NTP高延迟/抖动及-500ppm调节，不能据“synchronized=yes”假定短时不漂移。
.18 NTP inactive，实采GO2(.18)-PC偏差中位112.236ms，变化约-0.783ms/s。
这不是原生雷达时间校准；原生header/DDS发送时间域与PC差约3,390,496秒。
`.161`是主控候选；用户在可见终端尝试后确认没有登录凭据，未修改该主机。
对.161和.18的标准NTP只读请求均超时，未获得原生源时钟独立校准基准。
不能仅由DDS时间域推断已读到源主机系统时间或确认具体源IP。

脚本记录PC ROS/wall/monotonic（第二轮同时记录MONOTONIC_RAW）、原始与转换header、
PC接收时刻、DDS source timestamp、点内time极值、state/base，以及以base源stamp查询
map->vehicle的query_ns/tf_stamp_ns。已安装rclpy executor丢弃MessageInfo，因此诊断专用
executor保留元信息，不修改系统ROS库。Point-LIO增加默认关闭的time_diagnostic_log，
开启时记录实际now/beg/end/time_current/last_imu/odom；不改变内部数值、发布或滤波。

### 根因证据

官方L1接口把PointUnitree.time定义为相对cloud.stamp的时间：
[官方SDK声明](https://github.com/unitreerobotics/unilidar_sdk/blob/main/unitree_lidar_sdk/include/unitree_lidar_sdk.h)。
本机实时消息提供额外实证，而不是直接假定原生驱动等同SDK示例：原始点内time为正，
最大相对时间中位63.761ms，原生DDS发布在header后中位66.373ms；
`DDS source - (header+max time)`范围约0.205–5.051ms，中位2.599ms（含采集启动窗口）。
原始IMU的DDS source与其header只差约8微秒。因此当前流的header是点内相对时间参考
（扫描起点语义），完整点云发送在扫描末端后，与相对time语义一致。

旧transform_everything在首个完整点云回调取`PC now - raw header`为永久offset；
这个差同时包含不同epoch、完整扫描跨度、源端处理和单向传输/PC调度延迟，
不能视为纯时钟差。第二轮转换后cloud起点约在PC当前时刻，而转换后IMU已超前约75.6ms，
故最早引入额外未来偏差的环节是transform_everything；Point-LIO随后正常计算扫描末端。
源码位置：transform_everything.py的cloud_callback、imu_callback。

Point-LIO preprocess.cpp unilidar_handler把time秒乘1000存入curvature毫秒；
laserMapping.cpp sync_packages仅执行`end=beg+max(curvature)/1000`一次；
publish_odometry_without_downsample=false时发布该end，并原样用于动态TF。
实测内部end和odom最大差仅约0.238微秒（double转换精度），未发现重复加扫描时间。
从输入点云最大time推导的end与输出非全部精确匹配，是preprocess盲区过滤改变最大保留点；
诊断同时保留未过滤推导值及真正内部日志，不把推导值伪装成内部实测。
IMU_Processing.hpp未另行添加扫描周期；parameters.cpp时间lag为实测0。

base_odom_node.py仅坐标偏移、header复制；正式30秒446/446个base stamp在state中精确匹配，
446/446次按原始stamp查询TF返回相同tf_stamp。science_target_approach._fresh仍为100ms
future_tolerance，sample_time=monotonic-age，未来age会把采样时间映到将来，超过100ms即拒绝；
本次未改此逻辑。camera_target_to_map仍按输入PointStamped原始时间查TF，latest仅供诊断文字，
没有替代精确时间查询。没有把时间戳问题解释成SLAM空间漂移。

### 正式30秒基线统计

从记录的wall/monotonic样本对应关系切出正式窗口，启动前/暖机样本单独保留。
正值为未来；单位ms，趋势为ms/s。原始receive_delay含不同epoch，不能当网络延迟。

| 项目 | 样本 | 均值 | 中位 | P95 | 最小 | 最大 | 趋势 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 转换后cloud起点-PC | 445 | -2.857 | -2.686 | 7.795 | -26.072 | 23.412 | -0.745 |
| 转换后IMU-PC | 7481 | 75.298 | 75.580 | 86.644 | 41.678 | 89.064 | -0.752 |
| 实际内部lidar_end-PC | 462 | 57.665 | 58.187 | 68.560 | 31.254 | 72.803 | -0.810 |
| state-PC | 446 | 57.865 | 57.757 | 67.928 | 36.385 | 72.080 | -0.732 |
| base-PC | 446 | 57.412 | 57.215 | 67.361 | 35.944 | 71.510 | -0.729 |

raw cloud receive_delay中位3,390,496.116008s，raw IMU中位3,390,496.048432s；
二者相差约67.58ms；完整均值/极值/P95/斜率在measured_summary.json，不省略或当作纯网络时延。
首轮/第二轮分别观测到11/1条启动IMU未加offset，随后切换到巨大的epoch偏移，
证实启动阶段双时基问题。后续历史100ms故障日志没有同步逐级原始采样，
本次仅复现约50–80ms典型偏差，未宣称完整重现此前108ms峰值。

### 最小代码修正与真实阻塞

- 不再从首个完整点云的接收时刻估计时钟差。默认保留原生时间戳；可显式配置
  sensor_time_offset_sec，但必须来自独立实测映射，不提供猜测值。
- 首个cloud验证同一时基前不发IMU；cloud/IMU都应用同一明确映射。
  使用now只作时基有效性检查，不覆盖源stamp；不删除原始timestamp或点内时间。
- 源时基不可信时输出SOURCE_CLOCK_UNSYNCHRONIZED并阻断cloud/IMU。
  这不是自动完成源时钟同步：当前相差约39天、无独立基准，不能继续沿用伪造offset。
- 仅给Point-LIO增加默认关闭的只读日志和launch开关/显式offset传参；
  未改估计算法、滤波器、运动/外参/MAP接近参数、0.10s保护。
- 同一一键入口按源文件指纹只构建必要包，无变化不重复构建；不重启已运行SLAM。
  完整运动入口先执行此只读诊断，返回2时在机械臂使能/行走前停止。

修正后正式30秒采到445帧原始点云、7495条原始IMU；转换cloud/IMU及state/base均0条。
startup未换算IMU也为0条。节点日志明确给出源时钟未校准，没有输出虚假的“零未来误差”。
**源时钟实际同步尚未完成；修正后有效里程计偏差、长期漂移和定位精度均NOT VERIFIED。**
缺少源主机权限或可信双向同步接口时无法消除单向链路延迟与时钟差的不可辨识性，
不以minimum接收延迟等经验值冒充精确时钟标定。

### 入口、验证和数据

唯一实现入口：`bash camera_ws/scripts/run_go2_time_diagnostic.sh`。
根go2_test.sh的--time-diagnostic共用该实现；--execute在S1集成分支先诊断。
主目录入口转发独立集成目录，不切分支或覆盖已有脏改动；/tmp丢失可恢复该分支。
已实际完成Point-LIO单包构建26.3s、必要launch安装/转换包构建，以及只读前后采集；
后续仅语法/lint及实采统计分析，没有全仓构建、离线仿真或批量单元测试。

原始数据在主仓库camera_ws/records（不随Git上传）：
`time_20261112_230956_before`、`time_20261112_231501_before_internal`、
`time_20261112_231926_source_guard_after`、`time_20261112_232648_final_entry`。
保留samples.jsonl、metadata.json、readonly_slam.log、point_lio_internal.json、report.json/txt、trend.json。
可审阅统计及原始数据SHA256：`camera_ws/analysis/go2_time_sync_20261112/measured_summary.json`。
下一步需取得原生时钟基准后真正同步/验证；不能将当前阻断状态当完整解决。

## 2026-11-12后续 — 纯PC DDS发现及LidarState实采（无需SSH）

来源：fetch后的12d0db5、PC Domain0/enp12s0实际DDS发现、30秒真实消息、官方公开接口源码。
本轮完全没有SSH、运动/RPC请求、雷达配置/同步操作；没有构建，没有重启已有SLAM。
开始与结束节点列表相同，均包含已有laserMapping/transform_everything/3个static TF节点。
用户在本轮明确确认雷达型号为L1；LidarState没有型号字段，不把版本号推断冒充自动识别。

### 实际发现及读取

- /utlidar/cloud：sensor_msgs/msg/PointCloud2，30秒446帧。
- /utlidar/imu：sensor_msgs/msg/Imu，30秒7491条。
- /utlidar/lidar_state：unitree_go/msg/LidarState，发现后订阅，30秒149条，确实可读。
- 三个writer的participant prefix均为0110fa3127471848ca77a567，记录完整GID及实际发现列表。
- 状态实读firmware_version=1.0.2、software_version=1.0.0.38、sdk_version=1.0.9；整段版本不变。
  sdk_version是雷达驱动报告的SDK版本，不是声称PC安装的unitree_sdk2版本号。
- cloud_frequency字段范围14.478–15.101Hz，imu_frequency为247.935–250.662Hz；这是状态报告值，
  与采集样本计数分别保留。error_state全0。serial_buffer_size=0–240，serial_buffer_read=4–311。
- 本机ROS消息字段与[官方unitree_ros2 LidarState.msg](https://github.com/unitreerobotics/unitree_ros2/blob/master/cyclonedds_ws/src/unitree/unitree_go/msg/LidarState.msg)一致。
  [当前SDK2 LidarState_.hpp](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/idl/go2/LidarState_.hpp)
  另含dirty_percentage；本轮未更新消息包或硬套新IDL，按实际可解码的官方ROS2版本读取共同字段。
  官方unitree_ros2说明可直接用ROS消息与SDK2 DDS互通，本轮使用此路径，没有声称运行了新C++ SDK reader。
- 顺带只读查看实际发现的/rtc/state，仅收到connection_state=not_connected；这不是系统时钟证据。
  /rtc_status未收到样本，未把名字当成可用的主控校时服务。

### 时间结果

差值统一为“源字段减PC wall”，负值表示源时间早；PC回调wall只是比较时刻，
**不是拿DDS接收时间估计纯时钟offset**。本机rmw报告received_timestamp=0，原值保留，不伪造接收时刻。
samples.jsonl同时保留PC ROS、wall、monotonic、MONOTONIC_RAW以及DDS source timestamp。

| 字段 | 样本 | 与PC wall差的中位数(s) |
|---|---:|---:|
| raw cloud header | 446 | -3390496.118454 |
| raw cloud DDS source | 446 | -3390496.051751 |
| raw IMU header | 7491 | -3390496.050694 |
| raw IMU DDS source | 7491 | -3390496.050684 |
| LidarState.stamp | 149 | -3390496.050732 |
| LidarState DDS source | 149 | -3390496.050716 |
| LidarState.serial_recv_stamp | 149 | -3390496.051527 |

serial_recv_stamp-stamp中位-0.499ms，范围-3.386到+0.112ms；
DDS source-serial_recv_stamp中位+0.517ms，P95约1.974ms；
DDS source-LidarState.stamp中位+20.705微秒。
raw cloud DDS source-header中位66.372ms，raw IMU对应中位7.846微秒。
所有均值/P95/极值/趋势及首末值保存在report.json和可审阅摘要，不把单帧结果当整段固定关系。
新旧时间差缓慢变化，当前raw IMU source-PC斜率约-0.309ms/s，并非回放了39天前的静态消息。

按用户要求重新计算旧原始记录的DDS source-PC wall（包含启动样本，独立注明窗口）：
23:15那轮cloud/IMU中位分别-3390496.047387/-3390496.046293s；
23:19为-3390496.037933/-3390496.036979s；23:26为-3390495.866029/-3390495.865044s。
因此原始DDS发送时间本身也在旧时间域，不能把问题仅归给PC转换器，更不能把这个差当网络延迟。

### A/B/C根因判断及边界

已确认：误差在发布端时间域就存在；同一DDS participant下的header、串口接收标记、
LidarState stamp及DDS发送时间都同域，支持A（发布者主机/驱动所用时钟域）为主要方向。
B（仅LiDAR内部时钟错）不能单独解释DDS发送和串口接收标记也同样错。
C（生产者驱动显式把同一旧时间写入不同消息/DDS时间）仍不能在未获该实现或系统时间服务时完全排除。
未直接读取该主机OS日期，不把“最符合A”写成已通过date确认A。
[官方SDK2 DdsWriter](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/common/dds/dds_entity.hpp)
使用不带显式source timestamp的write；这支持正常SDK路径下源时间来自发布者时钟，
但不能证明当前封闭原生进程完全采用同一实现。

### 官方同步接口核对（未执行）

- [L1 unilidar_sdk头文件](https://github.com/unitreerobotics/unilidar_sdk/blob/main/unitree_lidar_sdk/include/unitree_lidar_sdk.h)：
  getTimeDelay()返回微秒级链路延迟，TIMESYNC是runParse返回的消息类别；
  两者不等于设置GO2主控系统UTC的DDS RPC。当前固件TIMESYNC实际收发未观测，不能由枚举存在声称支持已验证。
  要直接使用reader需访问同一雷达串口/专用UDP连接；当前只有机器人DDS，不抢占GO2现有驱动。
- [L2 unilidar_sdk2头文件](https://github.com/unitreerobotics/unilidar_sdk2/blob/main/unitree_lidar_sdk/include/unitree_lidar_sdk.h)：
  有syncLidarTimeStamp()，initializeSerial/UDP可选use_system_timestamp；这是L2直连reader接口，
  不适用于用户确认的L1，也不是通过DDS修改机器人主控时钟。本轮没有初始化L2连接或调用同步。
- [RobotStateClient](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/robot_state/robot_state_client.hpp)
  公开的是ServiceList/ServiceSwitch/SetReportFreq，未公开设置系统时间方法。
  [ConfigClient](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/config/config_client.hpp)
  是通用配置文件Set/Get/Meta接口，没有文档化主机时间键，不能猜键或用Set试探。
  实际发现的/api/bashrunner或/utlidar/client_cmd也没有在本次所查公开接口里找到校时定义，不发送猜测请求。

结论：已查公开接口中尚无当前L1/GO2 DDS连接可直接调用的主控校时方法。
真正下一步需要宇树针对firmware1.0.2/software1.0.0.38/SDK1.0.9确认并提供
**无需SSH的主控NTP/SNTP联网校时配置或官方维护/固件校时服务**，然后用同一命令验证。
不再把源主机登录作为前提，也不捏造App菜单、RobotState API或改PC日期迁就旧时间。
因此本轮完成DDS定位证据，尚未实际修复源时钟；没有将PC接收时间写回消息来冒充修复。

### 交付与证据

唯一入口仍是`bash camera_ws/scripts/run_go2_time_diagnostic.sh`；现为纯PC DDS订阅，
无SSH、无重建、无SLAM启停，发现不到LidarState则报告实际缺失。
本轮30秒原始记录为camera_ws/records/time_20261112_235022_pc_dds_lidar_state，
保留samples.jsonl、metadata.json、report.json/txt、trend.json。
可随Git审查的完整统计、发现列表、版本/缓冲字段和旧记录重算及原始数据SHA256：
camera_ws/analysis/go2_time_sync_20261112/pc_dds_lidar_state_summary.json。
仅执行目标脚本语法/lint和真实DDS订阅，无批量离线测试或全仓审查；稳定MAP分支未改。

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

## 2026-11-13 右侧物块横向净距调整（静态）

用户确认前后到位但右侧过近，要求机身再远离20～35cm；本轮取中值27.5cm。
集成停靠Y目标由S1右侧0.31m改为0.585m，机身停车位向左移0.275m；前后位置、
4cm/10°/0.5s、速度和S1动作链不变。几何校验允许停车目标在抓取参考区外，
原29～33cm抓取参考区保持，predicted_in_grasp_region=false不伪称可抓取。
默认DockSettings历史值保留，真实入口通过science_grasp_docking.yaml覆盖；计划输出同步。
新版仅定向软件检查，真实净距/机械臂可达性NOT VERIFIED；下一轮先用dock，不直接full。
命令：bash go2_test.sh --execute --mode dock（真实运动，本轮未执行）。

验证：26项停靠定向检查通过，go2_science_perception单包构建通过，py_compile/diff检查通过；无实机运动。

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

本轮构建：仅go2_science_perception构建成功，git diff --check通过；未启动硬件。

## 2026-11-13 SEARCH速度0.35m/s（用户指定）

SEARCH profile从0.25改0.35m/s，SEARCH独立上限校验改0.35；共享安全门max_speed改0.35以免截断。
接近/停靠控制器max_linear_speed仍0.30，微调最低目标0.10且制动优先，角速度及加减速不变。
SEARCH仍5m/60秒，停稳确认/唯一控制出口/定位异常停车不变；前移3cm尚未实施。
本轮仅软件修改，不启动运动，新速度真实行走与停止表现NOT VERIFIED。

验证：39项SEARCH/停靠检查通过，go2_science_perception单包构建成功，diff检查通过。未运动。清理本机遗留录像与监视进程；远端SSH拒绝/超时，远端残留未确认。
