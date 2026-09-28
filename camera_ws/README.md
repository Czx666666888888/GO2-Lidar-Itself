# Standalone D435 Perception Workspace

## Goal

为安装在 Unitree GO2 前端的 Intel RealSense D435 建立独立 ROS 2 Humble 实验环境。最终系统将识别科学目标、读取目标像素处的对齐深度、由相机内参反投影为三维点，并在完成外参标定后转换到 GO2 雷达 SLAM 的 `map` 坐标系。

## Current Scope

- 启动 D435 color、depth 和 aligned-depth-to-color 数据流。
- 检查 RGB、aligned depth、color/depth `CameraInfo` 是否持续到达。
- 输出 topic、消息类型、频率、`frame_id`、分辨率和 depth encoding。
- 从运行时 `CameraInfo` 输出 `fx/fy/cx/cy`，禁止硬编码相机内参。
- 完全独立于 `code/` 中的 Point-LIO、规划、safety gate 和 GO2 控制。

## Future Scope

- 目标检测与分类。
- 像素 `(u,v)` 对应的有效 aligned-depth 读取与异常值处理。
- 像素和深度反投影为 camera optical frame 的 `(Xc,Yc,Zc)`。
- 发布带准确 `header.frame_id` 的 `geometry_msgs/msg/PointStamped`。
- 标定 camera frame 到 GO2 body/base frame 的外参。
- 使用 TF2 转换到 SLAM `map` frame，并交给后续导航。

上述内容均为 `Future integration / NOT IMPLEMENTED`。

## Environment

2026-09-28 安装与实机验证结果：

- USB 已枚举 `8086:0b07 Intel Corp. RealSense D435`，`serial redacted`，`uvcvideo` 已绑定。
- `ros-humble-librealsense2`：`2.58.4-1jammy.20260830.174155`。
- `ros-humble-realsense2-camera`：`4.58.4-1jammy.20260908.100350`。
- `ros-humble-realsense2-camera-msgs`：`4.58.4-1jammy.20260907.215954`。
- `ros-humble-realsense2-description`：`4.58.4-1jammy.20260908.002050`。
- 包来源为 ROS 2 Ubuntu Jammy 官方 APT 仓库；wrapper 报告 librealsense build/runtime 均为 `2.58.4`。
- D435 固件为 `5.12.7.150`，USB descriptor 为 `3.2`。

RealSense ROS 官方文档要求 SDK 和 ROS wrapper 各选择一种安装来源，避免并存版本冲突。对于已经配置 ROS 2 Humble APT 源的 Ubuntu 22.04，官方提供的二进制安装形式为：

```bash
sudo apt install ros-humble-librealsense2\*
sudo apt install ros-humble-realsense2-\*
```

本机已按用户授权完成上述官方二进制包安装。官方来源：

- https://github.com/realsenseai/realsense-ros#installation-on-ubuntu
- https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md

## Build

安装 ROS 依赖后：

```bash
cd /home/czx666/today_full_20260901/today_full/camera_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`camera_ws/build/`、`install/` 和 `log/` 由仓库根 `.gitignore` 排除。

## Launch

```bash
cd /home/czx666/today_full_20260901/today_full/camera_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch go2_science_perception camera_test.launch.py
```

launch 使用 `config/realsense_d435.yaml` 启动官方 wrapper，并使用 `config/diagnostics.yaml` 配置两个只读检查节点。它不启动任何 GO2 或导航节点。

单独运行检查节点：

```bash
ros2 run go2_science_perception camera_diagnostics --ros-args \
  --params-file src/go2_science_perception/config/diagnostics.yaml
ros2 run go2_science_perception camera_info_inspector --ros-args \
  --params-file src/go2_science_perception/config/diagnostics.yaml
```

实时查看 RGB、aligned depth 并点击测距：

```bash
ros2 launch go2_science_perception depth_viewer.launch.py
```

窗口包括 `D435 RGB` 和 `D435 Aligned Depth`。在 RGB 窗口左键点击后，节点会从同一像素附近的 5×5 aligned-depth ROI 中排除 0、负值和非有限值，输出有效像素数以及米制 median。按 `q` 或 `Esc` 退出。节点会读取并打印实际 color `CameraInfo`，本阶段不计算 XYZ，也不发布机器人或导航命令。

## Topics

以下为本轮实测接口：

| Topic | Type | Purpose |
|---|---|---|
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | RGB image |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | Raw rectified depth |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | Depth aligned to RGB pixels |
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Color intrinsics |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Depth intrinsics |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Camera-internal static transforms |

## Expected Output

`camera_diagnostics` 每5秒逐流输出 topic、类型、累计消息数、估计频率、`frame_id`、分辨率、encoding 和 freshness。典型 depth encoding 预计为 `16UC1`，但必须以实际消息为准。

`camera_info_inspector` 收到第一条 color `CameraInfo` 后输出：

- `fx`, `fy`, `cx`, `cy`
- `width`, `height`
- `distortion_model`
- `frame_id`

## Known Limitations

- RGB、raw depth、aligned depth、color/depth CameraInfo 和相机内部静态 TF 均已收到真实消息；这不等于深度精度或长期稳定性已经标定。
- 640x480x30 profile 启动成功，小消息 metadata 和 CameraInfo 约 30 Hz；本机 Python 大图像订阅观测值较低（RGB 约 18 Hz、aligned depth 约 8 Hz），且 aligned depth 曾短暂超过 2 秒未更新后恢复。端到端图像吞吐稳定性仍需排查。
- RGB 与 aligned depth 均为 640x480，aligned depth 使用 color optical frame；视觉抽样显示两者有对应场景且深度非全零，但尚未做定量像素对齐/深度精度验证。
- wrapper 使用硬件时钟并报告其可能周期性复位；后续跨传感器时间同步需要专项验证。
- D435 不提供 IMU；配置显式禁用 gyro、accel 和 motion，不设计任何 IMU 依赖。
- 已实现像素 ROI 深度读取，但尚无目标检测、三维点发布、外参或 map 转换。
- 相机 optical、GO2 body 和 map 坐标严禁混用，参见 `docs/architecture.md`。

D400 系列型号能力依据官方 datasheet；带 IMU 的对应型号是 D435i，而不是本项目的 D435：
https://cdrdv2-public.intel.com/841984/Intel-RealSense-D400-Series-Datasheet.pdf
