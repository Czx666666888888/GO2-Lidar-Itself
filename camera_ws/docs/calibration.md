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

Status: CameraInfo implementation exists; live values are `NOT VERIFIED` because the ROS wrapper/SDK is not installed.

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
