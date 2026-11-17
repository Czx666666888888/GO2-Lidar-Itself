"""High-level task coordinator: explore → detect → nav → approach → grasp → loop."""

import json
import math
import time
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Int32, String


# ── State machine ────────────────────────────────────────────────────────────

class State(Enum):
    IDLE = auto()
    EXPLORE = auto()
    NAV_TO_TARGET = auto()
    WAIT_NAV_ARRIVE = auto()
    APPROACH = auto()
    GRASP = auto()
    RESUME_EXPLORE = auto()
    DONE = auto()
    FAULT = auto()


# published on /mission/motion_mode; consumed by MotionArbiter
MODE_IDLE = 'IDLE'
MODE_NAV = 'NAV'
MODE_APPROACH = 'APPROACH'

# approach result strings from /science/approach/status
_STOP_STATES = {'STOP', 'FAULT', 'MISSED', 'COMPLETED'}
_SUCCESS_STATES = {'COMPLETED', 'STOP'}   # either counts as "handed off"

# distance (m) from robot to target below which we skip FAR and go straight to approach
APPROACH_DIRECT_M = 1.6

# how close (m) to the FAR goal we consider "arrived"
NAV_ARRIVE_M = 1.2

# standoff distance forwarded to FAR as the nav goal
NAV_STANDOFF_M = 1.4


class ExplorationWaypoints:
    """Generate a simple expanding-arc sequence of exploration waypoints."""

    def __init__(self, step_m: float = 3.0, arc_deg: float = 60.0,
                 layers: int = 4, revisit_radius: float = 1.5):
        self.step = step_m
        self.arc = math.radians(arc_deg)
        self.layers = layers
        self.revisit_radius = revisit_radius
        self._visited: List[Tuple[float, float]] = []
        self._queue: List[Tuple[float, float]] = []

    def _build_queue(self, ox: float, oy: float, heading: float):
        pts = []
        for layer in range(1, self.layers + 1):
            r = layer * self.step
            # centre, left, right alternating
            for angle_offset in [0.0, self.arc, -self.arc,
                                  2 * self.arc, -2 * self.arc]:
                a = heading + angle_offset
                pts.append((ox + r * math.cos(a), oy + r * math.sin(a)))
        # filter already-visited
        kept = []
        for p in pts:
            if all(math.hypot(p[0] - v[0], p[1] - v[1]) > self.revisit_radius
                   for v in self._visited):
                kept.append(p)
        self._queue = kept

    def next(self, robot_xy: Tuple[float, float],
             heading: float) -> Optional[Tuple[float, float]]:
        """Return next candidate; rebuilds queue when empty."""
        # purge candidates too close to visited
        self._queue = [p for p in self._queue
                       if all(math.hypot(p[0] - v[0], p[1] - v[1]) > self.revisit_radius
                              for v in self._visited)]
        if not self._queue:
            self._build_queue(robot_xy[0], robot_xy[1], heading)
        if not self._queue:
            return None
        # pick nearest unvisited candidate in forward half-plane
        forward = [p for p in self._queue
                   if math.cos(heading) * (p[0] - robot_xy[0]) +
                      math.sin(heading) * (p[1] - robot_xy[1]) > 0]
        pool = forward if forward else self._queue
        return min(pool, key=lambda p: math.hypot(p[0] - robot_xy[0],
                                                   p[1] - robot_xy[1]))

    def mark_visited(self, xy: Tuple[float, float]):
        self._visited.append(xy)
        self._queue = [p for p in self._queue
                       if math.hypot(p[0] - xy[0], p[1] - xy[1]) > self.revisit_radius]


# ── Node ─────────────────────────────────────────────────────────────────────

class MissionManager(Node):
    """Drive the full autonomous mission loop without touching hardware directly."""

    def __init__(self):
        super().__init__('mission_manager')

        # parameters
        self.declare_parameter('max_targets', 6)
        self.declare_parameter('nav_arrive_m', NAV_ARRIVE_M)
        self.declare_parameter('approach_direct_m', APPROACH_DIRECT_M)
        self.declare_parameter('nav_standoff_m', NAV_STANDOFF_M)
        self.declare_parameter('nav_timeout_sec', 60.0)
        self.declare_parameter('explore_dwell_sec', 5.0)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('map_frame', 'map')

        self._max_targets = int(self.get_parameter('max_targets').value)
        self._nav_arrive_m = float(self.get_parameter('nav_arrive_m').value)
        self._approach_direct_m = float(self.get_parameter('approach_direct_m').value)
        self._nav_standoff_m = float(self.get_parameter('nav_standoff_m').value)
        self._nav_timeout = float(self.get_parameter('nav_timeout_sec').value)
        self._explore_dwell = float(self.get_parameter('explore_dwell_sec').value)
        self._dry_run = bool(self.get_parameter('dry_run').value)
        self._map_frame = str(self.get_parameter('map_frame').value)

        # state
        self._state = State.IDLE
        self._state_entered = time.monotonic()
        self._robot_xy: Optional[Tuple[float, float]] = None
        self._robot_heading: float = 0.0
        self._tracks: Dict[int, dict] = {}      # id -> track dict from manager
        self._completed_ids: set = set()
        self._active_target_id: Optional[int] = None
        self._active_target_pos: Optional[Tuple[float, float]] = None
        self._nav_goal_xy: Optional[Tuple[float, float]] = None
        self._approach_result: Optional[str] = None
        self._grasp_done = False
        self._explore_wp = ExplorationWaypoints()
        self._total_handled = 0

        # publishers
        self._pub_mode = self.create_publisher(String, '/mission/motion_mode', 1)
        self._pub_goal = self.create_publisher(PointStamped, '/goal_point', 1)
        self._pub_approach_start = self.create_publisher(Empty, '/science/approach/start', 1)
        self._pub_approach_confirm = self.create_publisher(
            Int32, '/science/approach/confirm_target_id', 1)
        self._pub_approach_reset = self.create_publisher(Empty, '/science/approach/reset', 1)
        self._pub_visited = self.create_publisher(Int32, '/science/visited_target_id', 1)
        self._pub_status = self.create_publisher(String, '/mission/status', 10)

        # subscriptions
        self.create_subscription(
            String, '/science/target_tracks', self._tracks_cb, 10)
        self.create_subscription(
            Odometry, '/base_state_estimation', self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(
            String, '/science/approach/status', self._approach_status_cb, 10)
        self.create_subscription(
            Empty, '/mission/start', self._start_cb, 1)
        self.create_subscription(
            Empty, '/mission/stop', self._stop_cb, 1)

        self.create_timer(0.5, self._tick)
        self._log(f'MissionManager ready; dry_run={self._dry_run}; state=IDLE')

    # ── callbacks ────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._robot_xy = (p.x, p.y)
        self._robot_heading = yaw

    def _tracks_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, KeyError):
            return
        self._tracks = {t['id']: t for t in data.get('tracks', [])
                        if t.get('state') == 'CONFIRMED'
                        and t['id'] not in self._completed_ids}

    def _approach_status_cb(self, msg: String):
        try:
            status = json.loads(msg.data)
            self._approach_result = status.get('state', '')
        except (json.JSONDecodeError, KeyError):
            pass

    def _start_cb(self, _msg):
        if self._state in (State.IDLE, State.DONE, State.FAULT):
            self._transition(State.EXPLORE)

    def _stop_cb(self, _msg):
        self._log('STOP command received')
        self._set_mode(MODE_IDLE)
        self._transition(State.IDLE)

    # ── helpers ───────────────────────────────────────────────────────────────

    def _log(self, text: str):
        self.get_logger().info(text)

    def _transition(self, new: State):
        self._log(f'state {self._state.name} -> {new.name}')
        self._state = new
        self._state_entered = time.monotonic()
        self._pub_status.publish(String(data=json.dumps({
            'state': new.name,
            'completed': self._total_handled,
            'active_target': self._active_target_id,
        })))

    def _elapsed(self) -> float:
        return time.monotonic() - self._state_entered

    def _set_mode(self, mode: str):
        self._pub_mode.publish(String(data=mode))

    def _publish_goal(self, x: float, y: float):
        msg = PointStamped()
        msg.header.frame_id = self._map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = x
        msg.point.y = y
        msg.point.z = 0.0
        self._pub_goal.publish(msg)

    def _robot_to_target_dist(self, tx: float, ty: float) -> Optional[float]:
        if self._robot_xy is None:
            return None
        return math.hypot(self._robot_xy[0] - tx, self._robot_xy[1] - ty)

    def _pick_target(self) -> Optional[dict]:
        """Return nearest unhandled confirmed track."""
        if not self._tracks or self._robot_xy is None:
            return None
        candidates = [t for t in self._tracks.values()
                      if t['id'] not in self._completed_ids]
        if not candidates:
            return None
        return min(candidates,
                   key=lambda t: math.hypot(
                       t['position'][0] - self._robot_xy[0],
                       t['position'][1] - self._robot_xy[1]))

    def _standoff_xy(self, tx: float, ty: float) -> Tuple[float, float]:
        """Point self._nav_standoff_m from target toward robot."""
        if self._robot_xy is None:
            return tx, ty
        dx = self._robot_xy[0] - tx
        dy = self._robot_xy[1] - ty
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return tx + self._nav_standoff_m, ty
        s = self._nav_standoff_m / norm
        return tx + dx * s, ty + dy * s

    # ── state machine tick ────────────────────────────────────────────────────

    def _tick(self):
        if self._robot_xy is None:
            return  # no localization yet

        if self._state == State.IDLE:
            self._set_mode(MODE_IDLE)

        elif self._state == State.EXPLORE:
            self._tick_explore()

        elif self._state == State.NAV_TO_TARGET:
            self._tick_nav_to_target()

        elif self._state == State.WAIT_NAV_ARRIVE:
            self._tick_wait_arrive()

        elif self._state == State.APPROACH:
            self._tick_approach()

        elif self._state == State.GRASP:
            self._tick_grasp()

        elif self._state == State.RESUME_EXPLORE:
            # brief pause then back to explore
            if self._elapsed() > 2.0:
                self._transition(State.EXPLORE)

        elif self._state == State.DONE:
            self._set_mode(MODE_IDLE)

        elif self._state == State.FAULT:
            self._set_mode(MODE_IDLE)

    # ── EXPLORE ───────────────────────────────────────────────────────────────

    def _tick_explore(self):
        self._set_mode(MODE_NAV)

        # check if any confirmed target found
        target = self._pick_target()
        if target is not None:
            tx, ty = target['position'][0], target['position'][1]
            dist = self._robot_to_target_dist(tx, ty)
            self._log(f'EXPLORE: found target id={target["id"]} dist={dist:.2f}m')
            self._active_target_id = target['id']
            self._active_target_pos = (tx, ty)
            if dist is not None and dist <= self._approach_direct_m:
                # close enough — go straight to approach
                self._approach_result = None
                self._pub_approach_reset.publish(Empty())
                self._transition(State.APPROACH)
            else:
                self._transition(State.NAV_TO_TARGET)
            return

        if self._total_handled >= self._max_targets:
            self._log('Max targets handled; mission DONE')
            self._transition(State.DONE)
            return

        # send next exploration waypoint every dwell interval or on first entry
        if self._elapsed() > self._explore_dwell or self._elapsed() < 0.6:
            wp = self._explore_wp.next(self._robot_xy, self._robot_heading)
            if wp is not None:
                self._publish_goal(wp[0], wp[1])
                self._log(f'EXPLORE: waypoint ({wp[0]:.1f}, {wp[1]:.1f})')
                self._state_entered = time.monotonic()  # reset dwell timer

    # ── NAV_TO_TARGET ─────────────────────────────────────────────────────────

    def _tick_nav_to_target(self):
        self._set_mode(MODE_NAV)

        if self._active_target_pos is None:
            self._transition(State.EXPLORE)
            return

        tx, ty = self._active_target_pos
        sx, sy = self._standoff_xy(tx, ty)
        self._nav_goal_xy = (sx, sy)

        # keep publishing goal at 2 Hz
        if self._elapsed() % 0.5 < 0.5:
            self._publish_goal(sx, sy)

        dist_to_goal = (math.hypot(self._robot_xy[0] - sx, self._robot_xy[1] - sy)
                        if self._robot_xy else 999.0)

        if dist_to_goal <= self._nav_arrive_m:
            self._log(f'NAV: arrived near target id={self._active_target_id}')
            self._transition(State.WAIT_NAV_ARRIVE)
            return

        # timeout
        if self._elapsed() > self._nav_timeout:
            self._log(f'NAV: timeout reaching id={self._active_target_id}; marking missed')
            self._completed_ids.add(self._active_target_id)
            self._active_target_id = None
            self._active_target_pos = None
            self._total_handled += 1
            self._transition(State.RESUME_EXPLORE)

    # ── WAIT_NAV_ARRIVE ───────────────────────────────────────────────────────

    def _tick_wait_arrive(self):
        """Stop NAV motion and hand off to APPROACH."""
        self._set_mode(MODE_IDLE)   # stop nav vel
        if self._elapsed() > 1.0:  # 1 s settle
            self._approach_result = None
            self._pub_approach_reset.publish(Empty())
            self._transition(State.APPROACH)

    # ── APPROACH ──────────────────────────────────────────────────────────────

    def _tick_approach(self):
        self._set_mode(MODE_APPROACH)

        # on first tick: send start + confirm target ID
        if self._elapsed() < 0.6:
            self._pub_approach_start.publish(Empty())
            if self._active_target_id is not None:
                self._pub_approach_confirm.publish(
                    Int32(data=self._active_target_id))
            return

        result = self._approach_result
        if result in _STOP_STATES:
            success = result in _SUCCESS_STATES
            self._log(f'APPROACH result={result} target={self._active_target_id}')
            if success:
                # hand off to grasp; arbiter keeps APPROACH mode
                self._transition(State.GRASP)
            else:
                self._completed_ids.add(self._active_target_id)
                self._total_handled += 1
                self._pub_visited.publish(Int32(data=self._active_target_id))
                self._active_target_id = None
                self._active_target_pos = None
                self._transition(State.RESUME_EXPLORE)

    # ── GRASP ─────────────────────────────────────────────────────────────────

    def _tick_grasp(self):
        # Motion control is owned by the grasp/S1 task; arbiter stays APPROACH
        # (no nav velocity can interfere). We just watch for the approach node
        # to reach a terminal state after grasp completes.
        result = self._approach_result
        if result in {'COMPLETED', 'MISSED', 'FAULT'}:
            self._log(f'GRASP cycle ended; result={result}')
            self._completed_ids.add(self._active_target_id)
            self._total_handled += 1
            self._pub_visited.publish(Int32(data=self._active_target_id))
            self._active_target_id = None
            self._active_target_pos = None
            self._set_mode(MODE_IDLE)
            if self._total_handled >= self._max_targets:
                self._transition(State.DONE)
            else:
                self._transition(State.RESUME_EXPLORE)
            return

        # safety: if grasp takes too long (S1 task timeout is 300 s)
        if self._elapsed() > 320.0:
            self._log('GRASP: timeout; marking missed')
            self._completed_ids.add(self._active_target_id)
            self._total_handled += 1
            self._active_target_id = None
            self._active_target_pos = None
            self._set_mode(MODE_IDLE)
            self._transition(State.RESUME_EXPLORE)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
