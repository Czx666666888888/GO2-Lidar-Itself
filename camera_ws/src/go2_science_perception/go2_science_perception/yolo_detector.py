"""Independent configurable YOLO RGB-D inference; no HSV/navigation publishers."""

from collections import deque
import hashlib
import json
from pathlib import Path
import signal
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .yolo_geometry import back_project, reliable_box_depth


def stamp_ns(header):
    """Return the exact image source time as an integer."""
    return header.stamp.sec*1_000_000_000+header.stamp.nanosec


def names_mapping(value):
    """Normalize checkpoint classes without any task-specific names."""
    if isinstance(value, list):
        value = dict(enumerate(value))
    return {int(k): str(v) for k, v in value.items()}


class YoloDetector(Node):
    """Process the latest RGB once; nearest source-timestamp depth is optional."""

    def __init__(self):
        """Load a local detection model and independent read-only topics."""
        super().__init__('yolo_detector')
        defaults = dict(
            model_path='', metadata_path='', class_names='', confidence=0.25, iou=0.45,
            imgsz=640, device='0', inference_rate_hz=10.0,
            color_topic='/camera/camera/color/image_raw',
            depth_topic='/camera/camera/aligned_depth_to_color/image_raw',
            camera_info_topic='/camera/camera/color/camera_info',
            max_rgb_depth_delta_sec=0.10, max_image_age_sec=1.0,
            max_rgb_info_delta_sec=0.10,
            min_depth_m=0.15, max_depth_m=5.0, depth_roi_fraction=0.5,
            min_depth_pixels=20, min_valid_depth_ratio=0.6, max_depth_spread_m=0.08,
            publish_debug_image=True, publish_legacy_debug=False, log_path='')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {key: self.get_parameter(key).value for key in defaults}
        self._validate_config()
        path = Path(self.cfg['model_path']).expanduser().resolve()
        if not path.is_file() or path.suffix not in ('.pt', '.onnx'):
            raise ValueError('model_path must be an existing local .pt or .onnx detection model')
        self.model_sha256 = hashlib.sha256(path.read_bytes()).hexdigest()
        # Optional heavy dependency: only imported by this new node.
        from ultralytics import YOLO
        import torch
        self.torch = torch
        self.use_cuda = self.cfg['device'].lower() != 'cpu' and torch.cuda.is_available()
        self.model = YOLO(str(path), task='detect')
        self.predict_args = dict(imgsz=self.cfg['imgsz'], device=self.cfg['device'],
                                 conf=self.cfg['confidence'], iou=self.cfg['iou'],
                                 verbose=False)
        self.model.predict(np.zeros((self.cfg['imgsz'], self.cfg['imgsz'], 3),
                                    dtype=np.uint8), **self.predict_args)
        if self.model.task != 'detect':
            raise ValueError('Expected a detection model')
        self.names = names_mapping(self.model.names)
        self._validate_metadata(path)
        selected = [s.strip() for s in self.cfg['class_names'].split(',') if s.strip()]
        unknown = set(selected)-set(self.names.values())
        if unknown:
            raise ValueError(f'Unknown selected classes: {sorted(unknown)}')
        self.class_ids = ([i for i, name in self.names.items() if name in selected]
                          if selected else None)
        self.bridge, self.rgb, self.info = CvBridge(), None, None
        self.depths = deque(maxlen=30)
        self.last_stamp = None
        self.log_stream = None
        if self.cfg['log_path']:
            log = Path(self.cfg['log_path']).expanduser()
            log.parent.mkdir(parents=True, exist_ok=True)
            self.log_stream = log.open('a', buffering=1)
        self.compat_debug = (self.create_publisher(String, '/science/detection_debug', 10)
                             if self.cfg['publish_legacy_debug'] else None)
        self.pub2d = self.create_publisher(String, '/science/yolo/detections', 10)
        self.pubpoint = self.create_publisher(PointStamped, '/science/yolo_target_point', 10)
        self.pubdebug = self.create_publisher(Image, '/science/yolo/debug_image', 2)
        self.create_subscription(Image, self.cfg['color_topic'], self._rgb,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, self.cfg['depth_topic'], self._depth,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.cfg['camera_info_topic'], self._info,
                                 qos_profile_sensor_data)
        self.create_timer(1.0/self.cfg['inference_rate_hz'], self._tick)
        self.get_logger().info(f'YOLO names={self.names}; isolated outputs; no navigation')

    def _validate_config(self):
        """Fail configuration errors before creating any runtime publisher."""
        c = self.cfg
        if not (0 < c['confidence'] <= 1 and 0 < c['iou'] <= 1):
            raise ValueError('confidence/iou must be in (0,1]')
        if c['imgsz'] <= 0 or c['imgsz'] % 32 or c['inference_rate_hz'] <= 0:
            raise ValueError('Positive rate and input size multiple of 32 required')
        if not (0 < c['depth_roi_fraction'] <= 1 and 0 < c['min_valid_depth_ratio'] <= 1):
            raise ValueError('Invalid depth ROI/ratio')
        if not (0 < c['min_depth_m'] < c['max_depth_m'] and c['min_depth_pixels'] > 0
                and c['max_rgb_depth_delta_sec'] >= 0 and c['max_depth_spread_m'] > 0
                and c['max_image_age_sec'] > 0 and c['max_rgb_info_delta_sec'] >= 0):
            raise ValueError('Invalid depth/timestamp limits')

    def _validate_metadata(self, path):
        """Bind export sidecar, bytes, class mapping and resolution when available."""
        explicit = self.cfg['metadata_path']
        sidecar = Path(explicit) if explicit else path.parent / 'metadata.json'
        if explicit or sidecar.is_file():
            metadata = json.loads(sidecar.read_text())
            if (metadata['sha256'] != self.model_sha256 or
                    names_mapping(metadata['names']) != self.names or
                    metadata['imgsz'] != self.cfg['imgsz'] or
                    metadata.get('task') != 'detect'):
                raise ValueError('Model bytes/classes/imgsz disagree with export metadata')
        else:
            self.get_logger().warning('No export metadata; names read from checkpoint only')

    def _rgb(self, msg):
        """Retain only the latest image rather than accumulating inference backlog."""
        self.rgb = msg

    def _depth(self, msg):
        """Keep a bounded source-time buffer for optional depth association."""
        self.depths.append(msg)

    def _info(self, msg):
        """Retain calibrated color-image projection information."""
        self.info = msg

    def _depth_context(self, rgb, shape):
        """Return depth only for compatible timestamps, frames, encodings and sizes."""
        if not self.depths:
            return None, None, 'no_depth'
        msg = min(self.depths, key=lambda d: abs(stamp_ns(d.header)-stamp_ns(rgb.header)))
        delta = (stamp_ns(rgb.header)-stamp_ns(msg.header))*1e-9
        if abs(delta) > self.cfg['max_rgb_depth_delta_sec']:
            return None, msg, 'rgb_depth_time_mismatch'
        if self.info is None:
            return None, msg, 'no_camera_info'
        if abs(stamp_ns(rgb.header)-stamp_ns(self.info.header))*1e-9 > self.cfg['max_rgb_info_delta_sec']:
            return None, msg, 'rgb_camera_info_time_mismatch'
        if stamp_ns(msg.header) <= 0:
            return None, msg, 'zero_depth_stamp'
        if (not rgb.header.frame_id or rgb.header.frame_id != msg.header.frame_id or
                rgb.header.frame_id != self.info.header.frame_id):
            return None, msg, 'optical_frame_mismatch'
        if (msg.height, msg.width) != shape or (self.info.height, self.info.width) != shape:
            return None, msg, 'depth_camera_size_mismatch'
        depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough').astype(np.float32)
        if msg.encoding in ('16UC1', 'mono16'):
            depth *= 0.001
        elif msg.encoding != '32FC1':
            return None, msg, 'unsupported_depth_encoding'
        return depth, msg, ''

    def _tick(self):
        """Publish per-image 2D detections, optional 3D and debug with source header."""
        rgb = self.rgb
        if rgb is None or stamp_ns(rgb.header) == self.last_stamp:
            return
        self.last_stamp = stamp_ns(rgb.header)
        start = time.perf_counter()
        age = (self.get_clock().now().nanoseconds-self.last_stamp)*1e-9
        report = dict(schema_version=1, rgb_stamp_ns=self.last_stamp,
                      frame_id=rgb.header.frame_id, width=rgb.width, height=rgb.height,
                      model_sha256=self.model_sha256, names=self.names, detections=[],
                      rgb_age_sec=age, inference_ms=None, rgb_depth_delta_sec=None,
                      depth_stamp_ns=None, status='OK',
                      camera_info_stamp_ns=stamp_ns(self.info.header) if self.info else None,
                      camera_info_frame_id=self.info.header.frame_id if self.info else None)
        try:
            if self.last_stamp <= 0 or age > self.cfg['max_image_age_sec'] or age < -0.1:
                report['status'] = 'stale_or_future_rgb'
                return
            image = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
            if self.use_cuda:
                self.torch.cuda.synchronize()
            infer_start = time.perf_counter()
            result = self.model.predict(image, classes=self.class_ids, **self.predict_args)[0]
            if self.use_cuda:
                self.torch.cuda.synchronize()
            report['inference_ms'] = (time.perf_counter()-infer_start)*1000
            report['ultralytics_speed_ms'] = result.speed
            try:
                depth, depth_msg, reason = self._depth_context(rgb, image.shape[:2])
            except Exception as exc:
                depth, depth_msg, reason = None, None, f'depth_conversion_error: {exc}'
            if depth_msg is not None:
                report['depth_frame_id'] = depth_msg.header.frame_id
                report['depth_stamp_ns'] = stamp_ns(depth_msg.header)
                report['rgb_depth_delta_sec'] = (self.last_stamp-report['depth_stamp_ns'])*1e-9
            for index, box in enumerate(result.boxes):
                cls = int(box.cls.item())
                bounds = [float(v) for v in box.xyxy[0].cpu().tolist()]
                detection = dict(index=index, class_id=cls, class_name=self.names[cls],
                                 confidence=float(box.conf.item()), bbox_xyxy=bounds)
                quality = dict(valid=False, reason=reason)
                if depth is not None:
                    c = self.cfg
                    quality = reliable_box_depth(
                        depth, bounds, c['min_depth_m'], c['max_depth_m'],
                        c['depth_roi_fraction'], c['min_depth_pixels'],
                        c['min_valid_depth_ratio'], c['max_depth_spread_m'])
                if quality['valid']:
                    completed_age = (self.get_clock().now().nanoseconds-self.last_stamp)*1e-9
                    if completed_age > self.cfg['max_image_age_sec']:
                        quality = dict(quality, valid=False, reason='inference_result_too_old')
                if quality['valid']:
                    try:
                        xyz = back_project(quality['u'], quality['v'], quality['depth_m'],
                                           self.info.k, self.info.d, self.info.distortion_model)
                        detection['xyz_m'] = xyz
                        point = PointStamped()
                        point.header = rgb.header
                        point.point.x, point.point.y, point.point.z = xyz
                        self.pubpoint.publish(point)
                    except (ValueError, cv2.error) as exc:
                        quality = dict(quality, valid=False, reason=f'invalid_intrinsics: {exc}')
                detection['depth'] = quality
                report['detections'].append(detection)
            if self.cfg['publish_debug_image']:
                canvas = result.plot()
                for d in report['detections']:
                    q = d['depth']; xy = d.get('xyz_m')
                    label = ('camera XYZ ' + ','.join(f'{v:.3f}' for v in xy)
                             if xy else 'depth: '+q.get('reason', 'invalid'))
                    u, v = int(d['bbox_xyxy'][0]), int(d['bbox_xyxy'][3])
                    cv2.putText(canvas, label, (max(0,u), min(canvas.shape[0]-8,v+16)),
                                cv2.FONT_HERSHEY_SIMPLEX,.4,(0,255,255),1)
                debug = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
                debug.header = rgb.header
                self.pubdebug.publish(debug)
        except Exception as exc:
            report['status'] = f'inference_error: {type(exc).__name__}: {exc}'
            self.get_logger().error(report['status'])
        finally:
            report['processing_ms'] = (time.perf_counter()-start)*1000
            encoded = json.dumps(report, allow_nan=False)
            self.pub2d.publish(String(data=encoded))
            if self.compat_debug is not None:
                targets = []
                for d in report['detections']:
                    x1,y1,x2,y2 = d['bbox_xyxy']; q=d['depth']
                    targets.append(dict(accepted=q['valid'], area=(x2-x1)*(y2-y1),
                        center=[q.get('u',(x1+x2)/2),q.get('v',(y1+y2)/2)],
                        contours=[[[x1,y1],[x2,y1],[x2,y2],[x1,y2]]],
                        class_id=d['class_id'],confidence=d['confidence'], depth=q))
                self.compat_debug.publish(String(data=json.dumps(dict(
                    color_stamp_ns=self.last_stamp, detector='yolo_color_block', targets=targets))))
            if self.log_stream:
                self.log_stream.write(encoded+'\n')


def main(args=None):
    """Start only the isolated detector; model changes require a node restart."""
    rclpy.init(args=args)
    node = None
    try:
        node = YoloDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ros2 launch and the terminal may both deliver SIGINT during cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            if node.log_stream:
                node.log_stream.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
