"""Read-only RGB-D dataset capture, using bounded source-timestamp buffers."""
from collections import deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np


def stamp_ns(msg):
    """Keep exact ROS source timestamps, without substituting reception time."""
    return int(msg.header.stamp.sec)*10**9 + int(msg.header.stamp.nanosec)


def nearest(messages, stamp):
    """Select the nearest buffered source stamp; arrival order is irrelevant."""
    return min(messages, key=lambda m: abs(stamp_ns(m)-stamp)) if messages else None


def camera_info_dict(info):
    """Serialize the actual calibration message and its source timestamp."""
    return dict(stamp_ns=stamp_ns(info), frame_id=info.header.frame_id,
                width=info.width, height=info.height, k=list(info.k), d=list(info.d),
                r=list(info.r), p=list(info.p), distortion_model=info.distortion_model,
                binning_x=info.binning_x, binning_y=info.binning_y,
                roi=dict(x_offset=info.roi.x_offset, y_offset=info.roi.y_offset,
                         width=info.roi.width, height=info.roi.height,
                         do_rectify=info.roi.do_rectify))


def save_sample(directory, rgb, depth, metadata):
    """Store lossless BGR PNG, native numeric NPY depth and a JSON sidecar."""
    folder = Path(directory)
    folder.mkdir(parents=True, exist_ok=False)
    if not cv2.imwrite(str(folder / 'rgb.png'), rgb):
        raise IOError('Could not save RGB PNG')
    if depth is not None:
        # No scaling, quantization or color mapping; retain uint16/float32 verbatim.
        np.save(folder / 'depth.npy', depth, allow_pickle=False)
    (folder / 'sample.json').write_text(json.dumps(
        dict(metadata, rgb_file='rgb.png', rgb_saved_encoding='bgr8',
             depth_file='depth.npy' if depth is not None else None),
        indent=2, allow_nan=False))


def sample_metadata(rgb, depth_msg, info, config, session, index):
    """Assess source-time/geometry association without changing depth values."""
    stamp = stamp_ns(rgb)
    flags = []
    meta = dict(schema_version=1, session_id=session, scene=config['scene'],
                source=config['color_topic'], source_video=None, frame_index=index,
                rgb_stamp_ns=stamp, rgb_frame_id=rgb.header.frame_id,
                rgb_encoding=rgb.encoding, rgb_width=rgb.width, rgb_height=rgb.height,
                depth_topic=config['depth_topic'],
                depth_stamp_ns=None, rgb_depth_delta_sec=None, camera_info=None,
                camera_info_delta_sec=None, depth_encoding=None,
                depth_unit=None, depth_scale_to_m=None)
    if depth_msg is None:
        flags.append('missing_depth')
    else:
        delta = (stamp-stamp_ns(depth_msg))*1e-9
        meta.update(depth_stamp_ns=stamp_ns(depth_msg), rgb_depth_delta_sec=delta,
                    depth_frame_id=depth_msg.header.frame_id,
                    depth_width=depth_msg.width, depth_height=depth_msg.height,
                    depth_encoding=depth_msg.encoding)
        if abs(delta) > config['max_rgb_depth_delta_sec']:
            flags.append('rgb_depth_time_mismatch')
        if (depth_msg.height, depth_msg.width) != (rgb.height, rgb.width):
            flags.append('depth_size_mismatch')
        if not rgb.header.frame_id or depth_msg.header.frame_id != rgb.header.frame_id:
            flags.append('aligned_optical_frame_mismatch')
        encoding = depth_msg.encoding
        if encoding in ('16UC1', 'mono16', '32FC1'):
            meta.update(depth_unit='mm' if encoding != '32FC1' else 'm',
                        depth_scale_to_m=0.001 if encoding != '32FC1' else 1.0)
        else:
            flags.append('unsupported_depth_encoding')
        if stamp_ns(depth_msg) <= 0:
            flags.append('invalid_depth_stamp')
    if info is None:
        flags.append('missing_camera_info')
    else:
        meta['camera_info'] = camera_info_dict(info)
        meta['camera_info_delta_sec'] = (stamp-stamp_ns(info))*1e-9
        if abs(meta['camera_info_delta_sec']) > config['max_info_delta_sec']:
            flags.append('camera_info_time_mismatch')
        if ((info.height, info.width) != (rgb.height, rgb.width) or
                info.header.frame_id != rgb.header.frame_id):
            flags.append('camera_info_geometry_mismatch')
        if not np.isfinite(info.k).all() or info.k[0] <= 0 or info.k[4] <= 0:
            flags.append('invalid_camera_intrinsics')
        if stamp_ns(info) <= 0:
            flags.append('invalid_info_stamp')
    if stamp <= 0:
        flags.append('invalid_rgb_stamp')
    meta.update(flags=flags, geometry_usable=not flags,
                pairing='nearest_source_stamp_after_bounded_wait')
    return meta


def main(args=None):
    """Subscribe only to RGB, aligned depth and color CameraInfo; publish nothing."""
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    class RGBDCapture(Node):
        def __init__(self):
            super().__init__('rgbd_capture')
            defaults = dict(
                output_directory='camera_ws/records/rgbd', session_id='', scene='',
                color_topic='/camera/camera/color/image_raw',
                depth_topic='/camera/camera/aligned_depth_to_color/image_raw',
                camera_info_topic='/camera/camera/color/camera_info',
                max_rgb_depth_delta_sec=0.10, max_info_delta_sec=0.50,
                pair_wait_sec=0.20, sample_interval_sec=1.0, duration_sec=30.0)
            for k, v in defaults.items():
                self.declare_parameter(k, v)
            self.c = {k: self.get_parameter(k).value for k in defaults}
            for k in ('max_rgb_depth_delta_sec', 'max_info_delta_sec', 'pair_wait_sec',
                      'sample_interval_sec', 'duration_sec'):
                if not math.isfinite(self.c[k]) or self.c[k] <= 0:
                    raise ValueError(f'{k} must be finite and positive')
            if '/aligned_depth_to_color/' not in self.c['depth_topic']:
                raise ValueError('Use the wrapper aligned_depth_to_color stream, not raw depth')
            session = self.c['session_id'] or datetime.now(timezone.utc).strftime(
                'rgbd_%Y%m%dT%H%M%S_%fZ')
            if Path(session).name != session or session in ('.', '..'):
                raise ValueError('session_id must be one directory name')
            if not self.c['scene'].strip():
                raise ValueError('Specify scene to preserve collection provenance')
            self.folder = Path(self.c['output_directory']).expanduser().resolve() / session
            self.folder.mkdir(parents=True, exist_ok=False)
            self.session = session
            (self.folder / 'session.json').write_text(json.dumps(dict(
                schema_version=1, session_id=session, configuration=self.c,
                created_utc=datetime.now(timezone.utc).isoformat()), indent=2))
            self.log = (self.folder / 'samples.jsonl').open('x', buffering=1)
            self.depths, self.infos, self.pending = deque(maxlen=90), deque(maxlen=90), deque()
            self.bridge = CvBridge()
            self.started = time.monotonic()
            self.next_capture = self.started
            self.frames = self.saved = self.accepted = 0
            self.done = False
            self.create_subscription(Image, self.c['color_topic'], self.on_rgb,
                                     qos_profile_sensor_data)
            self.create_subscription(Image, self.c['depth_topic'], self.depths.append,
                                     qos_profile_sensor_data)
            self.create_subscription(CameraInfo, self.c['camera_info_topic'], self.infos.append,
                                     qos_profile_sensor_data)
            self.create_timer(0.05, self.tick)
            self.get_logger().info(f'CAPTURE_READ_ONLY {self.folder}')

        def on_rgb(self, msg):
            index = self.frames
            self.frames += 1
            now = time.monotonic()
            if now < self.next_capture or now-self.started >= self.c['duration_sec']:
                return
            self.next_capture = now+self.c['sample_interval_sec']
            self.pending.append((now, index, msg))

        def record(self, index, rgb):
            stamp = stamp_ns(rgb)
            depth_msg, info = nearest(self.depths, stamp), nearest(self.infos, stamp)
            meta = sample_metadata(rgb, depth_msg, info, self.c, self.session, index)
            depth = None
            if depth_msg is not None and depth_msg.encoding in ('16UC1', 'mono16', '32FC1'):
                depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough').copy()
            folder = f'{self.saved:06d}_{stamp}'
            image = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
            save_sample(self.folder / folder, image, depth, meta)
            self.log.write(json.dumps(dict(meta, sample_directory=folder), allow_nan=False)+'\n')
            self.saved += 1
            self.accepted += meta['geometry_usable']

        def tick(self):
            now = time.monotonic()
            while self.pending and now-self.pending[0][0] >= self.c['pair_wait_sec']:
                _, index, msg = self.pending.popleft()
                self.record(index, msg)
            if now-self.started >= self.c['duration_sec']+self.c['pair_wait_sec']:
                self.done = True

    rclpy.init(args=args)
    node = None
    try:
        node = RGBDCapture()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            # Pending images are saved with the available candidates and explicit flags.
            while node.pending:
                _, index, msg = node.pending.popleft()
                node.record(index, msg)
            summary = dict(samples=node.saved, geometry_usable=node.accepted,
                           rgb_received=node.frames, session_id=node.session)
            (node.folder / 'summary.json').write_text(json.dumps(summary, indent=2))
            node.log.close()
            node.get_logger().info(f'CAPTURE_FINISHED {summary}')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
