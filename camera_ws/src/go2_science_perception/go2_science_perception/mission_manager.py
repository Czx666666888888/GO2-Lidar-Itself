"""Own FULL goals, validate docking handoff and authorize the existing controllers."""
from collections import Counter
from dataclasses import fields
import json
import math
import time

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
        self.association_radius = approach['science_target_approach']['ros__parameters'][
            'anchor_association_radius']
        self.completed = json.loads(self.args.completed_targets_json)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self.node)
        self.pose = None
        self.pose_ns = 0
        self.last_send = 0.
        self.cloud_ns = 0
        self.verified_pose_ns = 0
        self.raw_path = None
        self.raw_cloud = None
        self.track = None
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
            (Marker, '/viz_path_topic', self.far_path, 10),
            (PointStamped, '/mission/far_way_point', self.far_waypoint, 10),
            (Path, LOCAL_PATH_TOPIC, self.local_path, qos_profile_sensor_data),
        ):
            self.node.create_subscription(kind, topic, callback, qos)

    def now_ns(self):
        return self.node.get_clock().now().nanoseconds

    def fresh(self, ns, seconds):
        return ns > 0 and 0 <= self.now_ns() - ns < seconds * 1e9

    def common_origin(self):
        # localPlanner consumes numeric /way_point XY directly in Point-LIO's
        # origin. Reject map drift or a second map rather than reinterpret XY.
        try:
            transform = self.tf.lookup_transform('map', 'camera_init', Time()).transform
        except TransformException:
            return False
        t, q = transform.translation, transform.rotation
        if not (math.hypot(t.x, t.y, t.z) < .001 and math.hypot(q.x, q.y, q.z) < .001):
            return False
        if not self.pose:
            return False
        try:
            vehicle = self.tf.lookup_transform('map', 'vehicle',
                                               Time(nanoseconds=self.pose_ns)).transform
        except TransformException:
            return self.fresh(self.verified_pose_ns, .35)
        p = vehicle.translation
        if (math.dist((p.x, p.y), self.pose[:2]) >= .08
                or abs(wrap(yaw(vehicle.rotation) - self.pose[2])) >= .10):
            self.verified_pose_ns = 0
            return False
        self.verified_pose_ns = self.pose_ns
        return True

    def odom(self, message):
        ns = stamp(message.header)
        if not self.fresh(ns, .8) or message.header.frame_id not in ('map', 'camera_init'):
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

    def tracks(self, message):
        try:
            data = json.loads(message.data)
            if data['frame'] != 'map':
                self.track = None
                return
            tracks = [t for t in data['tracks']
                      if t['state'] == 'CONFIRMED' and not t['visited']
                      and self.fresh(t['observed_ns'], 1.2)
                      and len(t['position']) == 3 and all(math.isfinite(v) for v in t['position'])
                      and not any(math.dist(t['position'][:2], xy[:2]) < .15 for xy in self.completed)]
            distinct = [t for t in tracks if all(
                u['id'] == t['id'] or math.dist(u['position'][:2], t['position'][:2])
                >= 2 * self.association_radius for u in tracks)]
            tracks = [t for t in distinct if (t['id'] == self.anchor['id'] if self.anchor
                                              else self.can_retry_target(t['id']))]
            self.track = min(tracks, key=lambda t: math.dist(t['position'][:2], self.pose[:2]),
                             default=None) if self.pose else None
            if self.anchor:
                self.tracks_pub.publish(String(data=json.dumps({'frame': 'map', 'tracks': tracks})))
        except (KeyError, ValueError, TypeError):
            self.track = None

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

    def fail(self, reason):
        self.error = reason
        self.change('UNREACHABLE')
        self.run.event('MISSION_FAILURE', reason=reason)
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
        self.track = self.anchor = self.stage = None
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
        self.pending = options
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
        if self.pending:
            self.stage = self.pending.pop(0)
            self.send_goal((*self.stage.pose[:2], self.pose[3]))
            self.run.event('STAGING_PROBE', pose=self.stage.pose, side=self.stage.side)
            self.deadline = time.monotonic() + 2.4
        elif self.ranked:
            _, self.stage = min(self.ranked, key=lambda item: item[0])
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
                   if not any(math.dist(p.pose[:2], old.pose[:2]) < .13 for old in self.rejected)]
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
                and self.terrain.clear(pose, self.terrain.hard_clearance, starting_pose=pose)
                and self.terrain.sweep_clear(pose, docks[stage.side]))

    def command_clear(self, command):
        if self.arbiter.mode not in ('APPROACH', 'DOCK'):
            return True
        if not self.fresh(self.pose_ns, .55) or not self.fresh(self.cloud_ns, .85):
            return False
        twist = command.twist
        vx, vy, wz = twist.linear.x, twist.linear.y, twist.angular.z
        if not (vx or vy or wz):
            return True
        horizon = .45
        x, y, angle = self.pose[:3]
        dx, dy = rotate_xy(angle + .5*horizon*wz, (horizon*vx, horizon*vy))
        clear = self.terrain.sweep_clear(self.pose, (x+dx, y+dy, angle+horizon*wz))
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
        if not self.common_origin() or not self.fresh(self.pose_ns, .55):
            self.arbiter.enabled = False
            if time.monotonic() > self.startup_deadline:
                self.fail('Shared map TF or body odometry unavailable')
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
            if self.track and self.fresh(self.track['observed_ns'], 1.2):
                rejected = Counter()
                options = self.evaluate_staging(rejected)
                if options:
                    self.anchor = self.track
                    self.rejected = []
                    self.probe(options)
                    return
                self.defer_target('No staging pose passes the geometry and clearance checks',
                                  rejected=dict(rejected))
            if self.explore_goal and self.explore_goal[1] > self.goal_ns:
                self.send_goal(self.explore_goal[0])
            self.arbiter.enabled = bool(self.goal and self.live_path())
        elif self.state in ('PROBE', 'NAV_TO_TARGET', 'STOP_CONFIRM'):
            self.record_path()
            self.stage_step()
        elif self.state in ('APPROACH', 'DOCK'):
            self.record_path()
            report = self.run.control
            if report.get('state') in ('STOP', 'FAULT'):
                self.change('GRASP' if report.get('grasp_docking_complete') else 'IDLE',
                            'GRASP' if report.get('grasp_docking_complete') else 'IDLE')
                self.finished = True
            elif self.motion_blocked_at is not None and time.monotonic() - self.motion_blocked_at > 1.5:
                self.fail('UNREACHABLE: near-controller movement violates body clearance')
            elif (report.get('state') not in NEAR_STATES
                  or time.monotonic()-self.run.last_control_received > .55
                  or report.get('target_id') != self.anchor['id']
                  or report.get('grasp_side') != self.stage.side):
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

    def stage_step(self):
        now = time.monotonic()
        target_valid = (self.track and self.fresh(self.track['observed_ns'], 1.2)
                        and math.dist(self.track['position'][:2], self.anchor['position'][:2]) <= .12)
        if self.state == 'STOP_CONFIRM':
            if not target_valid or not self.stage_clear(self.pose):
                self.fail('UNREACHABLE: stop-confirm channel or target lost')
            report = self.run.control
            if report.get('state') in NEAR_STATES:
                if report.get('target_id') != self.anchor['id'] or report.get('grasp_side') != self.stage.side:
                    self.fail('STOP_CONFIRM selected a different identity or side')
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
                self.change('STOP_CONFIRM')
                hold = self.point((self.pose[0], self.pose[1], self.pose[3]))
                self.goal_pub.publish(hold)
                self.way_pub.publish(hold)
                self.run.start.publish(Empty())
                self.deadline = now + 12.
            elif now - self.last_progress > 5.:
                self.replan('Target observation or measured stop unavailable at staging')
        elif now - self.last_progress > 8.:
            self.replan('FAR/local path missing or staging progress stalled')
        else:
            self.drive(distance)
