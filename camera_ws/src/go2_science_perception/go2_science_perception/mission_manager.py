"""Own FULL goals, validate docking handoff and authorize the existing controllers."""
from collections import Counter, deque
from dataclasses import fields
import json
import math
import time

import numpy as np

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker
import yaml

from .docking_pose_planner import TerrainFootprint, staging_candidates
from .full_nbv_explore import return_tail_index
from .grasp_pose_geometry import DockSettings, docking_goal, rotate_xy, shared_side_settings, wrap
from .motion_arbiter import MotionArbiter
from .wall_distance import corner_grasp_heading, wall_grasp_side, wall_segments


LOCALIZATION_RECOVERY_TIMEOUT = 5.0
LOCALIZATION_STABLE_TIME = .30
LOCAL_PATH_TOPIC = '/nav/path'
NAV_TOPICS = ('/goal_point', '/way_point', LOCAL_PATH_TOPIC,
              '/nav/cmd_vel_raw', '/science/cmd_vel_raw')

NEAR_STATES = ('LOCK_TARGET', 'ALIGN', 'APPROACH_VISUAL', 'APPROACH_ODOM',
               'APPROACH_TO_GRASP_POSE')


def stamp(header):
    return Time.from_msg(header.stamp).nanoseconds


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


class MissionManager:
    """Select goals and gate motion, leaving FAR and local trajectories to their planners."""

    def __init__(self, run, config):
        self.run = run
        self.node = run.node
        self.args = run.args
        self.arbiter = MotionArbiter(self.node)
        self.arbiter.guard = self.command_clear
        self.terrain = TerrainFootprint(yaml.safe_load((config/'body_footprint.yaml').read_text()))
        overlay = yaml.safe_load((config/'science_grasp_docking.yaml').read_text())
        params = overlay['science_target_approach']['ros__parameters']
        geometry = DockSettings(**{f.name: params['dock_'+f.name] for f in fields(DockSettings)
                                   if 'dock_'+f.name in params})
        self.sides = shared_side_settings(geometry)
        if self.args.grasp_side != 'auto':
            self.sides = {self.args.grasp_side: self.sides[self.args.grasp_side]}
        approach = yaml.safe_load((config/'science_target_approach.yaml').read_text())
        approach_params = approach['science_target_approach']['ros__parameters']
        self.association_radius = approach_params['anchor_association_radius']
        self.memory_spread_limit = approach_params['anchor_max_spread']
        self.memory_range = approach_params['max_target_range']
        search = yaml.safe_load((config/'science_target_search.yaml').read_text())
        self.direct_range = search['science_target_approach']['ros__parameters']['search_trigger_range_m']
        self.completed = json.loads(self.args.completed_targets_json)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self.node)
        self.pose = None
        self.pose_ns = 0
        self.rejected_odom = None
        self.last_send = 0.
        self.cloud_ns = 0
        self.verified_pose_ns = 0
        self.localization_paused_at = None
        self.localization_recovered_at = None
        self.localization_issue = None
        self.raw_path = None
        self.raw_cloud = None
        self.track = None
        self.eligible_tracks = []
        self.camera_ranges = deque(maxlen=128)
        self.wall_points = np.empty((0, 2))
        self.last_direct_scan = 0
        self.direct_blockers = {}
        self.active_side = None
        self.side_hint = self.dock_heading = None
        self.memory_tracks = []
        self.memory_visit = None
        self.memory_visited = set()
        self.explore_goal = None
        self.goal = None
        self.goal_ns = 0
        self.far_cost = None
        self.far_ns = self.way_ns = self.local_ns = 0
        self.far_path_issue = 'far_path_not_received'
        self.local_path_issue = 'local_path_not_received'
        self.waypoint = None
        self.state = 'IDLE'
        self.pending = []
        self.ranked = []
        self.stage = None
        self.anchor = None
        self.deferred_targets = {}
        self.deadline = 0.
        self.last_progress = 0.
        self.best_distance = float('inf')
        self.had_path = False
        self.enabled = False
        self.error = None
        self.finished = False
        self.motion_blocked_at = None
        self.trail_file = self.args.navigation_trail
        self.trail = []
        self.return_index = None
        self.goal_pub = self.node.create_publisher(PointStamped, '/goal_point', 10)
        self.way_pub = self.node.create_publisher(PointStamped, '/way_point', 10)
        self.tracks_pub = self.node.create_publisher(String, '/mission/target_tracks', 10)
        self.mode_pub = self.node.create_publisher(String, '/mission/motion_mode', 10)
        self.stage_pub = self.node.create_publisher(Marker, '/mission/staging_pose', 10)
        self.target_pub = self.node.create_publisher(String, '/mission/goal', 10)
        for kind, topic, callback, qos in (
            (Odometry, '/base_state_estimation', self.odom, qos_profile_sensor_data),
            (PointCloud2, '/terrain_map', self.cloud, qos_profile_sensor_data),
            (PointStamped, '/mission/nbv_goal', self.nbv_goal, 10),
            (String, '/science/target_tracks', self.tracks, 10),
            (String, '/science/map_measurement_debug', self.camera_range, 10),
            (Marker, '/viz_path_topic', self.far_path, 10),
            (PointStamped, '/mission/far_way_point', self.far_waypoint, 10),
            (Path, LOCAL_PATH_TOPIC, self.local_path, qos_profile_sensor_data),
        ):
            self.node.create_subscription(kind, topic, callback, qos)

    def now_ns(self):
        return self.node.get_clock().now().nanoseconds

    def fresh(self, ns, seconds):
        return ns > 0 and 0 <= self.now_ns() - ns < seconds * 1e9

    def check_localization(self, *, allow_cached_tf):
        # localPlanner consumes numeric /way_point XY in Point-LIO's origin.
        try:
            transform = self.tf.lookup_transform('map', 'camera_init', Time()).transform
        except TransformException as exc:
            return dict(reason='map_camera_init_tf_unavailable', detail=str(exc))
        t, q = transform.translation, transform.rotation
        if not (math.hypot(t.x, t.y, t.z) < .001 and math.hypot(q.x, q.y, q.z) < .001):
            return dict(reason='map_camera_init_origin_mismatch')
        if self.pose is None:
            return dict(reason='body_odometry_missing', rejected_odom=self.rejected_odom)
        if not self.fresh(self.pose_ns, .55):
            age_sec = (self.now_ns() - self.pose_ns) / 1e9
            return dict(reason='body_odometry_timestamp_invalid', odom_age_sec=age_sec,
                        rejected_odom=self.rejected_odom)
        try:
            vehicle = self.tf.lookup_transform('map', 'vehicle',
                                               Time(nanoseconds=self.pose_ns)).transform
        except TransformException as exc:
            if allow_cached_tf and self.fresh(self.verified_pose_ns, .35):
                return None
            return dict(reason='map_vehicle_tf_at_odom_stamp_unavailable',
                        odom_stamp_ns=self.pose_ns, detail=str(exc))
        p = vehicle.translation
        xy_error = math.dist((p.x, p.y), self.pose[:2])
        yaw_error = abs(wrap(yaw(vehicle.rotation) - self.pose[2]))
        if not (xy_error < .08 and yaw_error < .10):
            self.verified_pose_ns = 0
            return dict(reason='body_odometry_tf_mismatch',
                        xy_error_m=xy_error if math.isfinite(xy_error) else None,
                        yaw_error_rad=yaw_error if math.isfinite(yaw_error) else None)
        self.verified_pose_ns = self.pose_ns
        return None

    def localization_ready(self, now):
        paused_at = self.localization_paused_at
        issue = self.check_localization(allow_cached_tf=paused_at is None)
        if issue and self.state not in ('EXPLORE', 'RETURN'):
            self.arbiter.enabled = False
            if now > self.startup_deadline:
                self.fail('Shared map TF or body odometry unavailable', localization=issue)
            return False
        if issue:
            self.localization_issue = issue
            self.localization_recovered_at = None
            if paused_at is None:
                self.localization_paused_at = paused_at = now
                self.arbiter.flush()
                self.run.event('LOCALIZATION_PAUSED', state=self.state, **issue)

        if paused_at is None:
            return True
        if now - paused_at >= LOCALIZATION_RECOVERY_TIMEOUT:
            self.fail('Shared map TF or body odometry did not recover',
                      localization=self.localization_issue, paused_sec=now - paused_at)
        if issue:
            return False

        if self.localization_recovered_at is None:
            self.localization_recovered_at = (now, self.pose_ns)
        ready_at, first_pose_ns = self.localization_recovered_at
        if now - ready_at < LOCALIZATION_STABLE_TIME or self.pose_ns <= first_pose_ns:
            return False

        self.arbiter.flush()
        self.last_progress += now - paused_at
        self.localization_paused_at = self.localization_recovered_at = None
        self.localization_issue = None
        self.run.event('LOCALIZATION_RECOVERED', state=self.state, paused_sec=now - paused_at)
        return True

    def odom(self, message):
        ns = stamp(message.header)
        received_age_sec = (self.now_ns() - ns) / 1e9
        if (ns <= 0 or not 0 <= received_age_sec < .8
                or message.header.frame_id not in ('map', 'camera_init')):
            self.rejected_odom = dict(stamp_ns=ns, age_sec=received_age_sec,
                                      frame=message.header.frame_id)
            return
        if ns < self.pose_ns:
            self.error = 'Map odometry timestamp regressed'
            return
        p, q = message.pose.pose.position, message.pose.pose.orientation
        pose = (p.x, p.y, yaw(q), p.z)
        if not all(math.isfinite(value) for value in pose):
            return
        if self.pose and math.dist(pose[:2], self.pose[:2]) > 1.:
            self.error = 'Map odometry discontinuity'
            return
        self.pose, self.pose_ns = pose, ns
        self.rejected_odom = None

    def cloud(self, message):
        if message.header.frame_id in ('map', 'camera_init') and self.fresh(stamp(message.header), 1.):
            self.raw_cloud = message

    def refresh_terrain(self):
        if self.raw_cloud is None:
            return
        now = time.monotonic()
        if now - getattr(self, 'terrain_at', -float('inf')) < .25:
            return
        message, self.raw_cloud = self.raw_cloud, None
        try:
            points = point_cloud2.read_points(message, field_names=('x', 'y', 'intensity'), skip_nans=True)
            self.terrain.update(points['x'], points['y'], points['intensity'])
            occupied = (points['intensity'] >= .20) & (points['intensity'] < 1.45)
            self.wall_points = np.column_stack((points['x'][occupied], points['y'][occupied]))
        except (AssertionError, ValueError, KeyError, TypeError):
            self.cloud_ns = 0
            return
        self.terrain_at = now
        self.cloud_ns = stamp(message.header)

    def nbv_goal(self, message):
        point = message.point
        if message.header.frame_id in ('map', 'camera_init') and all(
                math.isfinite(value) for value in (point.x, point.y, point.z)):
            self.explore_goal = ((point.x, point.y, point.z), stamp(message.header))

    def camera_range(self, message):
        try:
            data = json.loads(message.data)
            xyz, camera = data['map_xyz'], data['camera_xyz']
            ns = data['stamp_ns']
            if (data['target_frame'] == 'map' and self.fresh(ns, 1.2)
                    and len(xyz) == len(camera) == 3 and camera[2] > 0
                    and all(math.isfinite(v) for v in (*xyz, *camera))):
                self.camera_ranges.append((ns, tuple(xyz), math.hypot(*camera)))
        except (KeyError, ValueError, TypeError):
            return

    def camera_range_for(self, track):
        observation = track.get('latest_observation')
        if observation is None:
            return None
        return next((distance for ns, xyz, distance in reversed(self.camera_ranges)
                     if ns == track['observed_ns'] and math.dist(xyz, observation) <= 1e-6), None)

    def prioritize_grasp(self):
        """Claim the first fresh, wall-qualified target in the original approach range."""
        if self.last_direct_scan == self.cloud_ns:
            return False
        self.last_direct_scan = self.cloud_ns
        for target in self.eligible_tracks:
            if not self.fresh(target['observed_ns'], 1.2) or not self.can_retry_target(target['id']):
                continue
            distance = self.camera_range_for(target)
            if distance is None or distance > self.direct_range:
                self.direct_blocked(target, 'no_matching_rgbd' if distance is None
                                    else 'outside_camera_range', camera_range_m=distance)
                continue
            xy = target['position'][:2]
            candidates = [
                dict(distance_m=distance_m, wall_segment_map=endpoints,
                     side=wall_grasp_side(xy, endpoints, self.pose[2]),
                     eligible=.01 - 1e-6 <= distance_m <= .35 + 1e-6)
                for distance_m, endpoints in wall_segments(xy, self.wall_points)
            ]
            wall = next((candidate for candidate in candidates if candidate['eligible']), None)
            if wall is None:
                self.direct_blocked(target, 'outside_wall_range' if candidates else 'no_wall_segment',
                                    distance_m=candidates[0]['distance_m'] if candidates else None,
                                    wall_candidates=candidates)
                continue
            self.direct_blockers.pop(target['id'], None)
            self.anchor = self.track = target
            self.stage = self.active_side = None
            self.side_hint = wall['side'] if self.args.grasp_side == 'auto' else self.args.grasp_side
            alternatives = [c['wall_segment_map'] for c in candidates if c is not wall
                            and c['distance_m'] <= .55]
            corner = corner_grasp_heading(xy, wall['wall_segment_map'], alternatives, self.pose[2])
            self.dock_heading = corner[0] if corner else None
            self.memory_visit = None
            self.motion_blocked_at = None
            self.change('TARGET_STOP')
            self.hold_navigation()
            self.deadline = time.monotonic() + 5.
            self.run.event('DIRECT_GRASP_ADMITTED', target_id=target['id'],
                           target_map=xy, distance_m=wall['distance_m'],
                           wall_segment_map=wall['wall_segment_map'], wall_candidates=candidates,
                           camera_range_m=distance, side=self.side_hint,
                           corner_map=corner[1] if corner else None,
                           dock_heading_rad=self.dock_heading,
                           corner_turn_deg=round(math.degrees(wrap(self.dock_heading-self.pose[2])))
                           if corner else None)
            return True
        return False

    def direct_blocked(self, target, reason, **details):
        identity = target['id']
        if self.direct_blockers.get(identity) != reason:
            self.direct_blockers[identity] = reason
            self.run.event('DIRECT_GRASP_SKIPPED', target_id=identity, reason=reason,
                           target_map=target['position'][:2], **details)

    def tracks(self, message):
        try:
            data = json.loads(message.data)
            if data['frame'] != 'map':
                self.eligible_tracks = self.memory_tracks = []
                self.track = None
                return
            pending = [t for t in data['tracks'] if not t['visited']
                       and len(t['position']) == 3 and all(math.isfinite(v) for v in t['position'])
                       and not any(math.dist(t['position'][:2], xy[:2]) < .15 for xy in self.completed)]
            fresh = [t for t in pending if t['state'] == 'CONFIRMED'
                     and self.fresh(t['observed_ns'], 1.2)]
            tracks = [t for t in fresh if all(
                u['id'] == t['id'] or math.dist(u['position'][:2], t['position'][:2])
                >= 2*self.association_radius for u in fresh
            ) and (t['id'] == self.anchor['id'] if self.anchor else self.can_retry_target(t['id']))]
            self.eligible_tracks = sorted(tracks, key=lambda t: math.dist(t['position'][:2],
                                                                         self.pose[:2])) if self.pose else []
            self.track = self.eligible_tracks[0] if self.eligible_tracks else None
            self.memory_tracks = [t for t in pending if t['state'] == 'STALE'
                                  and t.get('memory_confirmed')
                                  and self.fresh(t['observed_ns'], 300.)
                                  and t.get('memory_spread_m', math.inf) < self.memory_spread_limit
                                  and all(u['id'] == t['id'] or
                                          math.dist(u['position'][:2], t['position'][:2])
                                          >= 2*self.association_radius for u in pending)]
            if self.anchor:
                hints = {}
                if self.side_hint:
                    hints['grasp_side_hint'] = self.side_hint
                if self.dock_heading is not None:
                    hints['dock_heading_hint'] = self.dock_heading
                if hints:
                    tracks = [dict(t, **hints) for t in tracks]
                self.tracks_pub.publish(String(data=json.dumps({'frame': 'map', 'tracks': tracks})))
        except (KeyError, ValueError, TypeError):
            self.eligible_tracks = self.memory_tracks = []
            self.track = None

    def revisit_memory(self):
        """Use the existing navigation for a view of a missed target in this MAP session."""
        now = time.monotonic()
        if self.memory_visit:
            target_id, deadline = self.memory_visit
            distance = math.dist(self.pose[:2], self.goal[:2])
            if now >= deadline or now-self.last_progress > 8.:
                self.run.event('MEMORY_REVISIT_END', target_id=target_id)
                self.memory_visit = None
                self.goal = None
                self.goal_ns = 0
                self.arbiter.flush()
            elif distance <= .30:
                self.arbiter.enabled = False
                self.memory_visit = (target_id, min(deadline, now+3.))
                return True
            else:
                self.drive(distance)
                return True

        for target in sorted(self.memory_tracks, key=lambda t: math.dist(t['position'][:2],
                                                                        self.pose[:2])):
            if target['id'] in self.memory_visited or not self.fresh(target['observed_ns'], 300.):
                continue
            px, py = self.pose[:2]
            tx, ty = target['position'][:2]
            distance = math.hypot(tx-px, ty-py)
            if not 1.15 < distance <= self.memory_range:
                continue
            # The view waypoint remains on the current side of the target.
            x, y = tx - .85*(tx-px)/distance, ty - .85*(ty-py)/distance
            if self.terrain.cells.get(self.terrain.cell(x, y)) != 1:
                continue
            self.memory_visited.add(target['id'])
            self.memory_visit = (target['id'], now+15.)
            self.send_goal((x, y, self.pose[3]))
            self.run.event('MEMORY_REVISIT', target_id=target['id'], xyz=(x, y),
                           target_map=(tx, ty))
            return True
        return False

    def change(self, state, mode='IDLE'):
        self.state = state
        self.arbiter.select(mode)
        self.mode_pub.publish(String(data=mode))
        self.run.event('MISSION_PHASE', state=state, mode=mode)

    def point(self, xyz):
        result = PointStamped()
        result.header.frame_id = 'map'
        result.header.stamp = self.node.get_clock().now().to_msg()
        result.point.x, result.point.y, result.point.z = map(float, xyz)
        return result

    def send_goal(self, xyz, direct=False):
        self.arbiter.flush()
        self.goal = xyz
        self.far_cost = self.waypoint = self.raw_path = None
        self.far_ns = self.way_ns = self.local_ns = 0
        self.far_path_issue = 'far_path_not_received'
        self.local_path_issue = 'local_path_not_received'
        self.last_progress = time.monotonic()
        self.best_distance = float('inf')
        self.had_path = False
        point = self.point(xyz)
        self.goal_ns = stamp(point.header)
        if direct:
            self.way_pub.publish(point)
            self.way_ns = self.goal_ns
        else:
            self.goal_pub.publish(point)
            self.last_send = time.monotonic()
        task_goal = dict(frame='map', state=self.state, xyz=xyz,
                         target_id=self.anchor['id'] if self.anchor else None)
        self.target_pub.publish(String(data=json.dumps(task_goal)))
        self.run.event('MISSION_GOAL', **task_goal)

    def far_path(self, message):
        ns = stamp(message.header)
        if (self.state not in ('EXPLORE', 'PROBE', 'NAV_TO_TARGET') or not self.goal
                or message.ns != 'global_path' or message.header.frame_id != 'map'
                or ns <= self.goal_ns):
            return
        points = [(p.x, p.y) for p in message.points]
        if (len(points) >= 2 and all(math.isfinite(v) for point in points for v in point)
                and math.dist(points[-1], self.goal[:2]) <= .18):
            self.far_ns = ns
            self.far_cost = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
            self.forward_waypoint()
        else:
            self.far_cost = None
            if len(points) < 2:
                self.far_path_issue = 'far_path_empty_or_too_short'
            elif not all(math.isfinite(v) for point in points for v in point):
                self.far_path_issue = 'far_path_nonfinite'
            else:
                self.far_path_issue = 'far_path_endpoint_outside_candidate'

    def far_waypoint(self, message):
        if (message.header.frame_id == 'map' and stamp(message.header) > self.goal_ns
                and all(math.isfinite(v) for v in (message.point.x, message.point.y, message.point.z))):
            self.waypoint = message
            self.forward_waypoint()

    def forward_waypoint(self):
        if (self.state not in ('EXPLORE', 'PROBE', 'NAV_TO_TARGET') or self.far_cost is None
                or not self.waypoint or abs(stamp(self.waypoint.header)-self.far_ns) > 200_000_000):
            return
        self.way_pub.publish(self.waypoint)
        if not self.way_ns:
            self.way_ns = stamp(self.waypoint.header)

    def local_path(self, message):
        if not self.raw_path or stamp(message.header) >= stamp(self.raw_path.header):
            self.raw_path = message

    def verify_local_path(self):
        message = self.raw_path
        if message is None:
            return
        ns = stamp(message.header)
        if (not self.way_ns or ns <= self.way_ns or message.header.frame_id != 'vehicle'
                or not self.fresh(ns, .65)):
            if self.way_ns:
                self.local_path_issue = 'local_path_wrong_frame_or_stamp'
            self.raw_path = None
            return
        if len(message.poses) < 2:
            self.local_path_issue = 'local_path_empty_or_too_short'
            self.local_ns = 0
            self.raw_path = None
            return
        try:
            transform = self.tf.lookup_transform('map', 'vehicle', Time.from_msg(message.header.stamp))
        except TransformException:
            self.local_path_issue = 'local_path_map_tf_unavailable'
            return
        self.raw_path = None
        p = message.poses[-1].pose.position
        if math.hypot(p.x, p.y) < .10 or not all(math.isfinite(v) for v in (p.x, p.y)):
            self.local_path_issue = 'local_path_endpoint_too_close_or_nonfinite'
            self.local_ns = 0
            return
        # The stamp-resolved TF binds the local path to the same map epoch.
        t = transform.transform.translation
        heading = yaw(transform.transform.rotation)
        rx, ry = rotate_xy(heading, (p.x, p.y))
        end = (t.x + rx, t.y + ry)
        if (self.pose and math.dist((t.x, t.y), self.pose[:2]) < .5
                and math.dist(end, self.pose[:2]) > .05):
            self.local_ns = ns
            self.local_path_issue = 'local_path_not_fresh_at_deadline'
        else:
            self.local_path_issue = 'local_path_inconsistent_with_body_pose'

    def live_path(self):
        return (self.fresh(self.local_ns, .65) and (self.state == 'RETURN' or
                self.far_cost is not None and self.fresh(self.far_ns, .9)))

    def fail(self, reason, **details):
        self.error = reason
        self.change('UNREACHABLE')
        self.run.event('MISSION_FAILURE', reason=reason, **details)
        raise RuntimeError(reason)

    def record_path(self):
        if not self.trail:
            if self.trail_file.exists():
                data = json.loads(self.trail_file.read_text())
                if (data['frame'] != 'map' or not data['points']
                        or data['last_ns'] >= self.pose_ns
                        or math.dist(data['points'][-1], self.pose[:2]) > .8):
                    self.fail('History is outside the current map epoch')
                self.trail = data['points']
            else:
                if self.args.return_only:
                    self.fail('RETURN has no recorded map trajectory')
                self.trail = [self.pose[:2]]
                self.save_path()
        if self.state != 'RETURN' and math.dist(self.trail[-1], self.pose[:2]) >= .28:
            self.trail.append(self.pose[:2])
            self.save_path()

    def save_path(self):
        temp = self.trail_file.with_suffix('.tmp')
        temp.write_text(json.dumps(dict(frame='map', last_ns=self.pose_ns, points=self.trail)))
        temp.replace(self.trail_file)

    def return_segment(self, index):
        # Keep corners in the recorded path and group only short straight segments.
        x, y = self.pose[:2]
        last = index
        for older in range(index - 1, max(-1, index - 5), -1):
            a, b = self.trail[older]
            length = math.hypot(a-x, b-y)
            if not .18 < length <= .90:
                break
            if any(abs((a-x)*(v-y) - (b-y)*(u-x))/length > .12
                   for u, v in self.trail[older+1:last+1]):
                break
            segment = (a, b, math.atan2(b-y, a-x))
            if not self.terrain.sweep_clear(self.pose, segment):
                break
            index = older
        return index

    def advance_return(self):
        while self.return_index >= 0:
            xy = self.trail[self.return_index]
            if math.dist(xy, self.pose[:2]) > (.35 if self.return_index == 0 else .16):
                self.return_index = self.return_segment(self.return_index)
                self.send_goal((*self.trail[self.return_index], self.pose[3]), direct=True)
                return
            self.return_index -= 1
        self.change('RETURN_SETTLE')
        self.deadline = time.monotonic() + 5.
        self.settled_at = None

    def can_retry_target(self, target_id):
        if target_id not in self.deferred_targets:
            return True
        position, retry_at = self.deferred_targets[target_id]
        return (time.monotonic() >= retry_at or
                self.pose is not None and math.dist(position, self.pose[:2]) >= .35)

    def defer_target(self, reason, **details):
        target = self.anchor or self.track
        self.deferred_targets[target['id']] = (self.pose[:2], time.monotonic() + 6.)
        self.run.event('TARGET_DEFERRED', status='UNREACHABLE', reason=reason,
                       target_id=target['id'], target_map=target['position'][:2],
                       vehicle_map=self.pose[:2], terrain_cells=len(self.terrain.cells), **details)
        self.track = self.anchor = self.stage = self.side_hint = self.dock_heading = None
        if self.state != 'EXPLORE':
            self.goal = None
            self.goal_ns = 0
            self.arbiter.flush()
            self.change('EXPLORE', 'NAV')

    def evaluate_staging(self, rejected):
        def log_candidate(**details):
            self.run.event('STAGING_CANDIDATE_REJECTED', target_id=self.track['id'],
                           target_map=self.track['position'][:2], terrain_stamp_ns=self.cloud_ns,
                           **details)

        return staging_candidates(self.track['position'][:2], self.pose, self.sides,
                                  self.terrain, rejected=rejected, on_reject=log_candidate)

    def probe(self, options):
        self.change('PROBE')
        self.stage = None
        self.ranked = []
        self.pending = sorted(options, key=lambda candidate: candidate.side != 'left')
        self.next_probe()

    def publish_stage(self):
        x, y, heading = self.stage.pose
        marker = Marker()
        marker.header = self.point((x, y, self.pose[3])).header
        marker.ns, marker.type, marker.action = 'full_staging', Marker.ARROW, Marker.ADD
        marker.pose.position.x, marker.pose.position.y = x, y
        marker.pose.position.z = self.pose[3]
        marker.pose.orientation.z, marker.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
        marker.scale.x, marker.scale.y, marker.scale.z = .5, .12, .12
        marker.color.g, marker.color.b, marker.color.a = .8, 1., 1.
        self.stage_pub.publish(marker)

    def next_probe(self):
        if self.pending and (not self.ranked or self.pending[0].side == self.ranked[0][1].side):
            side = self.stage.side if self.stage else None
            self.stage = self.pending.pop(0)
            if side == 'left' and self.stage.side == 'right':
                self.run.event('STAGING_SIDE_FALLBACK', target_id=self.anchor['id'],
                               reason='No left candidate has a valid FAR and local path')
            self.send_goal((*self.stage.pose[:2], self.pose[3]))
            self.run.event('STAGING_PROBE', pose=self.stage.pose, side=self.stage.side)
            self.deadline = time.monotonic() + 2.4
        elif self.ranked:
            cost, self.stage = min(self.ranked, key=lambda item: item[0])
            self.run.event('STAGING_SIDE_SELECTED', target_id=self.anchor['id'],
                           side=self.stage.side, far_path_length_m=cost)
            self.change('NAV_TO_TARGET', 'NAV')
            self.publish_stage()
            self.send_goal((*self.stage.pose[:2], self.pose[3]))
        else:
            self.defer_target('No staging candidate produced both a FAR route and a local path')

    def replan(self, reason):
        self.run.event('STAGING_REPLAN', reason=reason, target_id=self.anchor['id'],
                       pose=self.stage.pose, side=self.stage.side)
        self.rejected.append(self.stage)
        if not self.track or not self.fresh(self.track['observed_ns'], 1.2):
            self.defer_target('Target observation unavailable for replanning')
            return
        rejected = Counter()
        options = [p for p in self.evaluate_staging(rejected)
                   if not any(p.side == old.side and math.dist(p.pose[:2], old.pose[:2]) < .13
                              for old in self.rejected)]
        if options:
            self.probe(options)
        else:
            self.defer_target('No alternate staging pose passes the geometry and clearance checks',
                              rejected=dict(rejected))

    def stage_clear(self, pose):
        stage = self.stage
        target = self.track.get('latest_observation', self.track['position'])[:2]
        docks = {side: docking_goal(target, pose[2], settings)
                 for side, settings in self.sides.items()}
        chosen = min(docks, key=lambda side: (math.dist(docks[side][:2], pose[:2]), side != 'right'))
        return (chosen == stage.side
                and self.terrain.clear(stage.pose, self.terrain.parking_clearance)
                and self.terrain.clear(pose, self.terrain.grasp_clearance, starting_pose=pose)
                and self.terrain.sweep_clear(pose, docks[stage.side],
                                            clearance=self.terrain.grasp_clearance))

    def command_clear(self, command):
        if self.arbiter.mode not in ('APPROACH', 'DOCK'):
            return True
        if not self.fresh(self.pose_ns, .55) or not self.fresh(self.cloud_ns, .85):
            return False
        if self.stage is None:
            return True  # Wall-qualified direct grasp uses the original approach controller.
        twist = command.twist
        vx, vy, wz = twist.linear.x, twist.linear.y, twist.angular.z
        if not (vx or vy or wz):
            return True
        horizon = .45
        x, y, angle = self.pose[:3]
        dx, dy = rotate_xy(angle + .5*horizon*wz, (horizon*vx, horizon*vy))
        clear = self.terrain.sweep_clear(self.pose, (x+dx, y+dy, angle+horizon*wz),
                                         clearance=self.terrain.grasp_clearance)
        if clear:
            self.motion_blocked_at = None
        elif self.motion_blocked_at is None:
            self.motion_blocked_at = time.monotonic()
        return clear

    def stopped_for_handoff(self):
        inputs = self.run.control.get('stop_inputs', {})
        limits = dict(body_speed=.06, camera_speed=.08, sport_speed=.06, pose_yaw_rate=.12,
                      sport_yaw_rate=.12, odom_age=.35, camera_age=.35, sport_age=.35)
        return (time.monotonic()-self.run.last_control_received < .5 and
                all(inputs.get(k) is not None and abs(inputs[k]) <= v for k, v in limits.items()))

    def step(self):
        if not self.enabled:
            return
        if self.error:
            self.fail(self.error)
        conflicts = [topic for topic in NAV_TOPICS if self.node.count_publishers(topic) != 1]
        if conflicts:
            self.arbiter.enabled = False
            if time.monotonic() > self.startup_deadline:
                endpoints = {topic: [endpoint.node_namespace.rstrip('/')+'/'+endpoint.node_name
                                     for endpoint in self.node.get_publishers_info_by_topic(topic)]
                             for topic in conflicts}
                self.fail('Navigation requires one publisher per topic; found '+json.dumps(endpoints))
            return
        if not self.localization_ready(time.monotonic()):
            return
        if (self.state in ('EXPLORE', 'PROBE', 'NAV_TO_TARGET') and self.goal
                and not self.far_ns and time.monotonic() - self.last_send > 1.):
            self.goal_pub.publish(self.point(self.goal))
            self.last_send = time.monotonic()
        self.refresh_terrain()
        self.verify_local_path()
        if not self.fresh(self.cloud_ns, .85):
            self.arbiter.enabled = False
            if time.monotonic() > self.startup_deadline:
                self.fail('Shared map terrain stale')
            return
        if self.state == 'IDLE':
            self.record_path()
            if self.args.return_only:
                self.change('RETURN', 'RETURN')
                self.return_index = max(0, return_tail_index(self.trail, self.pose, .45))
                self.advance_return()
            else:
                self.change('EXPLORE', 'NAV')
        elif self.state == 'RETURN':
            if self.finished:
                return
            goal_distance = math.dist(self.pose[:2], self.goal[:2])
            if goal_distance <= (.35 if not self.return_index else .16):
                self.return_index -= 1
                self.advance_return()
            elif time.monotonic() - self.last_progress > 8.:
                self.fail('UNREACHABLE: recorded RETURN segment blocked')
            else:
                self.drive(goal_distance)
        elif self.state == 'RETURN_SETTLE':
            now = time.monotonic()
            if now > self.deadline or math.dist(self.pose[:2], self.trail[0]) > .35:
                self.fail('RETURN did not settle inside the map home radius')
            if not self.stopped_for_handoff():
                self.settled_at = None
            elif self.settled_at is None:
                self.settled_at = now
            elif now - self.settled_at > .4:
                self.change('RETURN_ARRIVED')
                self.finished = True
        elif self.state == 'EXPLORE':
            self.record_path()
            if self.prioritize_grasp() or self.revisit_memory():
                return
            if self.explore_goal and self.explore_goal[1] > self.goal_ns:
                self.send_goal(self.explore_goal[0])
            self.arbiter.enabled = bool(self.goal and self.live_path())
        elif self.state in ('PROBE', 'NAV_TO_TARGET', 'TARGET_STOP', 'STOP_CONFIRM'):
            self.record_path()
            self.stage_step()
        elif self.state in ('APPROACH', 'DOCK'):
            self.record_path()
            report = self.run.control
            if report.get('state') in ('STOP', 'FAULT'):
                self.change('GRASP' if report.get('grasp_docking_complete') else 'IDLE',
                            'GRASP' if report.get('grasp_docking_complete') else 'IDLE')
                self.finished = True
            elif (self.stage is not None and self.motion_blocked_at is not None
                  and time.monotonic() - self.motion_blocked_at > 1.5):
                self.fail('UNREACHABLE: near-controller movement violates body clearance')
            elif (report.get('state') not in NEAR_STATES
                  or time.monotonic()-self.run.last_control_received > .55
                  or report.get('target_id') != self.anchor['id']
                  or report.get('grasp_side') != self.active_side):
                self.fail('Original controller lost the selected handoff identity or side')
            else:
                if report.get('docking_phase') and self.state != 'DOCK':
                    self.change('DOCK', 'DOCK')
                self.arbiter.enabled = True

    def drive(self, distance):
        if distance + .10 < self.best_distance:
            self.best_distance, self.last_progress = distance, time.monotonic()
        self.arbiter.enabled = self.live_path()
        self.had_path |= self.arbiter.enabled

    def hold_navigation(self):
        hold = self.point((self.pose[0], self.pose[1], self.pose[3]))
        self.goal_pub.publish(hold)
        self.way_pub.publish(hold)
        self.goal = None

    def begin_stop_confirm(self, now):
        self.change('STOP_CONFIRM')
        self.hold_navigation()
        self.run.start.publish(Empty())
        self.deadline = now + 12.

    def stage_step(self):
        now = time.monotonic()
        target_valid = (self.track and self.fresh(self.track['observed_ns'], 1.2)
                        and math.dist(self.track['position'][:2], self.anchor['position'][:2]) <= .12)
        if self.state == 'TARGET_STOP':
            if target_valid and self.stopped_for_handoff():
                self.begin_stop_confirm(now)
            elif now >= self.deadline:
                self.defer_target('Target or measured stop unavailable for direct approach')
            return
        if self.state == 'STOP_CONFIRM':
            if not target_valid or (self.stage is not None and not self.stage_clear(self.pose)):
                self.fail('UNREACHABLE: stop-confirm channel or target lost')
            report = self.run.control
            if report.get('state') in NEAR_STATES:
                side = report.get('grasp_side')
                if (report.get('target_id') != self.anchor['id'] or side not in self.sides
                        or (self.stage is not None and side != self.stage.side)
                        or (self.side_hint is not None and side != self.side_hint)):
                    self.fail('STOP_CONFIRM selected a different identity or side')
                if (self.dock_heading is not None and
                        (report.get('stop_candidate') or {}).get('dock_heading_rad') != self.dock_heading):
                    self.fail('STOP_CONFIRM did not lock the requested corner heading')
                self.active_side = side
                self.run.event('GRASP_SIDE_LOCKED', target_id=self.anchor['id'], side=side)
                self.change('APPROACH', 'APPROACH')
            elif report.get('state') in ('STOP', 'FAULT') or now > self.deadline:
                self.fail('Original STOP_CONFIRM did not admit approach')
            return
        if self.state == 'PROBE':
            if self.live_path():
                self.ranked.append((self.far_cost, self.stage))
                # Bound wait once there are two genuinely reachable choices.
                if len(self.ranked) >= 2:
                    self.pending.clear()
                self.next_probe()
            elif now > self.deadline:
                if self.far_cost is None:
                    reason = self.far_path_issue
                elif not self.fresh(self.far_ns, .9):
                    reason = 'far_path_not_fresh_at_deadline'
                elif not self.way_ns:
                    reason = 'no_matching_far_waypoint_forwarded'
                else:
                    reason = self.local_path_issue
                self.run.event('STAGING_PROBE_TIMEOUT', target_id=self.anchor['id'],
                               pose=self.stage.pose, side=self.stage.side, reason=reason,
                               far_path=self.far_cost is not None, local_path=bool(self.local_ns))
                self.next_probe()
            return
        distance = math.dist(self.stage.pose[:2], self.pose[:2])
        at_stage = distance <= .24
        aligned = abs(wrap(self.pose[2] - self.stage.pose[2])) <= self.sides[self.stage.side].yaw_tolerance
        if at_stage and self.had_path:
            self.arbiter.enabled = False
            if not aligned or (target_valid and not self.stage_clear(self.pose)):
                self.replan('Parking heading or last approach segment invalid')
            elif target_valid and self.stopped_for_handoff():
                self.begin_stop_confirm(now)
            elif now - self.last_progress > 5.:
                self.replan('Target observation or measured stop unavailable at staging')
        elif now - self.last_progress > 8.:
            self.replan('FAR/local path missing or staging progress stalled')
        else:
            self.drive(distance)
