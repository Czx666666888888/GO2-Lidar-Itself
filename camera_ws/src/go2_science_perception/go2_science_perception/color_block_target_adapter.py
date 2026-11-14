"""One color-block PointStamped compatibility stream using the existing MAP transformer."""
import json
import math

from geometry_msgs.msg import PointStamped
import rclpy
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException

from .camera_target_to_map import CameraTargetToMap
from .color_block_selection import BlockSelection


class ColorBlockTargetAdapter(CameraTargetToMap):
    """Consume complete YOLO frames, select once in MAP and reject ambiguous updates."""

    def __init__(self):
        super().__init__()
        self.declare_parameter('max_observation_age_sec', .3)
        self.selection = BlockSelection()
        self.latest = None
        self.last_published = 0
        self.camera_point = self.create_publisher(PointStamped, '/science/target_coarse_point', 10)
        self.selection_debug = self.create_publisher(String, '/science/color_block/selection', 10)
        self.create_subscription(String, '/science/yolo/detections', self._frame, 10)
        self.create_timer(.02, self._process)

    def _point_callback(self, _message):
        # Base point subscription is deliberately inert: only complete YOLO frames enter.
        return

    def _frame(self, msg):
        try:
            report = json.loads(msg.data)
            if report['rgb_stamp_ns'] > self.last_published:
                self.latest = report
        except (ValueError, KeyError, TypeError):
            self.latest = None

    def _process(self):
        report = self.latest
        if report is None:
            return
        stamp = report.get('rgb_stamp_ns', 0)
        reason = 'invalid_report'
        diagnostic = dict(stamp_ns=stamp, selected=None, reason=reason)
        try:
            age = (self.get_clock().now().nanoseconds-stamp)/1e9
            if stamp <= self.last_published or not 0 <= age <= self.get_parameter('max_observation_age_sec').value:
                reason = 'stale_or_future_observation'
                self.latest = None
                return
            # A launch condition alone cannot protect against externally started HSV.
            for topic in ('/science/target_coarse_point', '/science/target_coarse_point_map'):
                owners = self.get_publishers_info_by_topic(topic)
                if len(owners) != 1 or owners[0].node_name != self.get_name():
                    reason = 'competing_formal_target_publisher'
                    return
            if self.count_publishers('/science/yolo/detections') != 1:
                reason = 'missing_or_competing_yolo_publisher'
                return
            if report.get('status') != 'OK' or not report.get('frame_id'):
                reason = 'invalid_detector_frame'
                return
            detections = [d for d in report['detections']
                          if d.get('class_id') == 0 and d.get('class_name') == 'color_block']
            if not detections:
                reason = 'no_color_block'
                return
            # An unmeasurable visible block could be nearer or be the locked block.
            if any(not d.get('depth', {}).get('valid') or 'xyz_m' not in d for d in detections):
                reason = 'color_block_depth_invalid'
                return
            template = PointStamped()
            template.header.frame_id = report['frame_id']
            template.header.stamp.sec, template.header.stamp.nanosec = divmod(stamp, 10**9)
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, template.header.frame_id,
                rclpy.time.Time.from_msg(template.header.stamp))
            camera_tf = self.tf_buffer.lookup_transform(
                self.map_frame, 'camera_link', rclpy.time.Time.from_msg(template.header.stamp))
            points, mapped = [], []
            for d in detections:
                xyz = d['xyz_m']
                if len(xyz) != 3 or xyz[2] <= 0 or not all(math.isfinite(x) for x in xyz):
                    reason = 'invalid_camera_xyz'
                    return
                point = PointStamped(); point.header = template.header
                point.point.x, point.point.y, point.point.z = xyz
                result = do_transform_point(point, transform)
                points.append(point)
                mapped.append((result.point.x, result.point.y, result.point.z))
            camera = camera_tf.transform.translation
            index, reason = self.selection.choose(mapped, (camera.x, camera.y))
            diagnostic.update(camera_map_xy=[camera.x, camera.y], candidates_map=mapped)
            if index is None:
                return
            # Reuse original exact-stamp transformer and original measurement diagnostics.
            self.input_count += 1
            self._transform_and_publish(points[index], 0.0)
            self.camera_point.publish(points[index])
            self.selection.commit(mapped[index])
            self.last_published = stamp
            self.latest = None
            diagnostic.update(selected=detections[index], camera_xyz=detections[index]['xyz_m'],
                              map_xyz=mapped[index], locked_initial=self.selection.initial)
        except TransformException:
            reason = 'waiting_for_exact_source_tf'
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            reason = 'invalid_frame: '+str(exc)
            self.latest = None
        finally:
            diagnostic['reason'] = reason
            self.selection_debug.publish(String(data=json.dumps(diagnostic, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = ColorBlockTargetAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
