# D435 Calibration Plan

## Intrinsic Calibration

The D435 factory calibration is exposed by the RealSense ROS wrapper through `sensor_msgs/msg/CameraInfo`.

Future pixel-to-3D code must read, for the exact active image profile:

- `K[0] = fx`
- `K[4] = fy`
- `K[2] = cx`
- `K[5] = cy`
- image width and height
- distortion model and coefficients
- `header.frame_id`

No D435 intrinsic value may be hard-coded. Changing resolution or stream profile may change the applicable calibration. This workspace provides `camera_info_inspector` to expose the runtime values.

2026-09-28 在 color 640x480 profile 下实测：`fx=608.898193359375`、`fy=608.2466430664062`、`cx=320.7738952636719`、`cy=243.91554260253906`，distortion model 为 `plumb_bob`，frame 为 `camera_color_optical_frame`。`camera_info_inspector` 输出与原始 CameraInfo 一致。这些值只适用于本次设备和 profile，未来代码仍必须运行时读取，不能复制为硬编码常量。

同次 raw depth CameraInfo 为 640x480，`fx=fy=386.96484375`、`cx=318.3269958496094`、`cy=235.739501953125`，frame 为 `camera_depth_optical_frame`。

## Extrinsic Calibration

The required future extrinsic is the rigid transform between the selected camera frame and the authoritative GO2 body/base frame:

```text
T_body_camera = [R_body_camera | t_body_camera]
```

It must capture the physical camera mounting translation and rotation. The optical frame axis convention must be handled explicitly; a tape-measured translation alone is insufficient.

Required future evidence:

- exact source and target frame names;
- mounting definition and axis diagram;
- calibration procedure and dataset;
- estimated translation/quaternion with units and convention;
- residual/error metrics;
- independent validation targets;
- versioned calibration file and hardware mounting identifier.

Status: `Future integration / NOT IMPLEMENTED`.

No static TF from camera to GO2 is published in this phase, and no relation to the existing SLAM `map` frame is claimed.
