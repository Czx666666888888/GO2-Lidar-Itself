"""Record actual RGB plus existing detector results and timestamped control logs."""

from datetime import datetime
import json
import os
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
from PIL import Image as PILImage, ImageDraw, ImageFont
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String


class ApproachVideoRecorder(Node):
    """Write MP4 and matching JSONL, including frames after visual loss."""

    def __init__(self):
        """Open a unique session and subscribe to the real camera/detector chain."""
        super().__init__('approach_video_recorder')
        self.declare_parameter('output_directory', 'camera_ws/records')
        self.declare_parameter('video_fps', 15.0)
        self.declare_parameter('post_stop_seconds', 3.0)
        folder = os.path.abspath(self.get_parameter('output_directory').value)
        os.makedirs(folder, exist_ok=True)
        name = 'approach_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.path = os.path.join(folder, name + '.mp4')
        self.log_path = os.path.join(folder, name + '.jsonl')
        self.log = open(self.log_path, 'x', buffering=1)
        self.writer = None
        self.raw_writer = None
        self.raw_path = os.path.join(folder, name + '_raw.mp4')
        self.frames = 0
        self.closed = False
        self.last_frame = -float('inf')
        self.first_stamp = self.last_stamp = None
        self.control, self.debug = {}, {}
        self.mask = None
        self.mask_stamp = 0
        self.terminal_at = None
        self.bridge = CvBridge()
        self.font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 17)
        self.health = self.create_publisher(String, '/science/approach/recording', 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self._rgb,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, '/science/detection_mask', self._mask,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/science/detection_debug', self._debug, 10)
        self.create_subscription(String, '/science/approach/status', self._control, 20)
        self.create_subscription(Empty, '/science/approach/finish_recording',
                                 lambda _: self.close(), 1)
        self.create_timer(0.5, self._heartbeat)
        self.get_logger().info(f'RECORDING_WAIT_RGB video={self.path} log={self.log_path}')

    def _write(self, kind, data):
        if not self.closed:
            self.log.write(json.dumps({'kind': kind,
                                       'record_ros_ns': self.get_clock().now().nanoseconds,
                                       **data}, allow_nan=False) + '\n')

    def _control(self, msg):
        self.control = json.loads(msg.data)
        self._write('control', {'control': self.control})
        if (self.control.get('state') in ('STOP', 'FAULT') and
                self.control.get('elapsed', 0) > 0 and self.terminal_at is None):
            self.terminal_at = time.monotonic()

    def _debug(self, msg):
        self.debug = json.loads(msg.data)
        self._write('detection', {'detection': self.debug})

    def _mask(self, msg):
        self.mask = self.bridge.imgmsg_to_cv2(msg, 'mono8').copy()
        self.mask_stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec

    @staticmethod
    def _point(point):
        return 'n/a' if point is None else f'({point[0]:+.3f}, {point[1]:+.3f})'

    def _rgb(self, msg):
        if self.closed:
            return
        stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8').copy()
        if self.writer is None:
            h, w = frame.shape[:2]
            self.writer = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*'mp4v'),
                                          self.get_parameter('video_fps').value, (w, h))
            self.raw_writer = cv2.VideoWriter(
                self.raw_path, cv2.VideoWriter_fourcc(*'mp4v'),
                self.get_parameter('video_fps').value, (w, h))
            if not self.raw_writer.isOpened():
                raise RuntimeError('Raw RGB MP4 writer could not open')
            if not self.writer.isOpened():
                raise RuntimeError('MP4 writer could not open')
            self.first_stamp = stamp
        self.last_frame = time.monotonic()
        self.last_stamp = stamp
        self.raw_writer.write(frame)
        age = (stamp-self.debug.get('color_stamp_ns', 0))/1e9
        fresh = 0 <= age <= 0.4
        if fresh:
            for target in self.debug.get('targets', []):
                color = (0, 255, 0) if target['accepted'] else (0, 160, 255)
                polygons = [np.asarray(p, dtype=np.int32).reshape(-1, 1, 2)
                            for p in target['contours'] if len(p) >= 3]
                cv2.drawContours(frame, polygons, -1, color, 2)
                uv = tuple(int(round(v)) for v in target['center'])
                cv2.drawMarker(frame, uv, color, cv2.MARKER_CROSS, 16, 2)
                cv2.putText(frame, f"{target['area']}px", (uv[0]+8, uv[1]-8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        c = self.control
        visible = (fresh and
                   any(t['accepted'] for t in self.debug.get('targets', [])))
        identity = c.get('target_id')
        if identity is None and c.get('candidate'):
            identity = c['candidate']['id']
        state = c.get('state', 'WAIT_CONTROL')
        distance = c.get('camera_distance_xy')
        dtext = 'n/a' if distance is None else f'{distance:.3f}m'
        map_point = c.get('target_map_anchor')
        if map_point is None and c.get('search_observations'):
            map_point = c['search_observations'][0]['map']
        lines = [f"ID={identity} DETECT={'VISIBLE' if visible else 'LOST'} {state}",
                 f"{'observed' if state == 'SEARCH' else 'anchor'} MAP "
                 f"{self._point(map_point)}",
                 f"camera MAP {self._point(c.get('camera'))} XY={dtext}",
                 f"vx={c.get('vx', 0):+.3f} wz={c.get('wz', 0):+.3f} detector age={age:.3f}s",
                 f'RGB stamp={stamp/1e9:.6f} frame={self.frames}']
        for index, text in enumerate(lines):
            y = 19 + 19*index
            cv2.putText(frame, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (255, 255, 255), 1, cv2.LINE_AA)
        if self.mask is not None:
            tile = cv2.cvtColor(cv2.resize(self.mask, (160, 120)), cv2.COLOR_GRAY2BGR)
            frame[-120:, -160:] = tile
            cv2.putText(frame, f'HSV age={(stamp-self.mask_stamp)/1e9:.2f}s',
                        (frame.shape[1]-158, frame.shape[0]-123),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        if state == 'APPROACH_ODOM':
            rgb = PILImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            ImageDraw.Draw(rgb).text((8, 115), 'TARGET LOST — MAP ODOM CONTROL',
                                     font=self.font, fill=(255, 190, 0),
                                     stroke_width=1, stroke_fill=(0, 0, 0))
            frame = cv2.cvtColor(np.asarray(rgb), cv2.COLOR_RGB2BGR)
        self.writer.write(frame)
        self._write('frame', {'frame_index': self.frames, 'rgb_stamp_ns': stamp,
                              'detector_age_sec': age, 'visible': visible,
                              'anchor_visual_visible': c.get('visual_visible'),
                              'control': c})
        self.frames += 1

    def _heartbeat(self):
        if (not self.closed and self.terminal_at is not None and
                time.monotonic()-self.terminal_at >=
                self.get_parameter('post_stop_seconds').value):
            self.close()
        self.health.publish(String(data=json.dumps({
            'active': not self.closed and time.monotonic()-self.last_frame < 0.5,
            'frames': self.frames, 'video': self.path, 'raw_video': self.raw_path,
            'jsonl': self.log_path,
            'last_rgb_stamp_ns': self.last_stamp, 'closed': self.closed})))

    def close(self):
        """Finalize MP4 metadata and retain the matching frame/control log."""
        if self.closed:
            return
        self._write('summary', {'frames': self.frames, 'first_rgb_stamp_ns': self.first_stamp,
                                'last_rgb_stamp_ns': self.last_stamp, 'video': self.path,
                                'raw_video': self.raw_path,
                                'nominal_fps': self.get_parameter('video_fps').value})
        self.closed = True
        if self.writer is not None:
            self.writer.release()
        if self.raw_writer is not None:
            self.raw_writer.release()
        self.log.close()
        self.get_logger().info(f'RECORDING_SAVED {self.path} frames={self.frames}')


def main(args=None):
    """Run the actual-camera recorder without any motion publisher."""
    rclpy.init(args=args)
    node = ApproachVideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
