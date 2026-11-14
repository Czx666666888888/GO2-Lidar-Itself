"""Optional body-frame planar docking through the existing velocity safety gate."""
import math

from .grasp_pose_geometry import DockSettings, geometry_report, rotate_xy
from .forward_step import ForwardStep


class GraspDocking:
    def __init__(self, settings=None, forward_profile=None):
        self.p = settings or DockSettings()
        self.forward = ForwardStep(forward_profile)
        self.reset()

    def reset(self):
        self.forward.reset()
        self.heading = None
        self.stationary_since = None
        self.stop_wait_started = None
        self.velocity_map = (0.0, 0.0)
        self.progress_at = None
        self.best_error = math.inf
        self.probe_origin = None
        self.probe_started = self.probe_zero = self.probe_stopped = None
        self.probe_zero_travel = None
        self.probe_result = {}

    def begin_response_check(self, core, now):
        self.probe_origin, self.probe_started = core.robot, now
        core.state = 'APPROACH_TO_GRASP_POSE'
        # This is measured travel from a real pose, not a fabricated visual anchor.
        core.target = core.anchor_initial = None

    def response_check(self, core, now, out, dt):
        """One 2cm measured trigger, 20cm/8s watchdog; no assumed pulse duration."""
        if self.probe_origin is None or core.state in ('IDLE', 'STOP', 'FAULT'):
            out.update(state=core.state, reason=core.reason, vx=0., vy=0., wz=0.,
                       grasp_docking_complete=False, response_check=self.probe_result,
                       control_reference='FORWARD_RESPONSE_CHECK')
            return out
        x, y, yaw = self.probe_origin
        travel, cross = rotate_xy(-yaw, (core.robot[0]-x, core.robot[1]-y))
        yaw_delta = math.atan2(math.sin(core.robot[2]-yaw), math.cos(core.robot[2]-yaw))
        if abs(cross) > .03 or abs(yaw_delta) > math.radians(3) or travel > .20:
            core.fault('forward response check path/heading boundary')
        if now-self.probe_started > 8.:
            core.fault('forward response check watchdog')
        if self.probe_zero is None and (travel >= .02 or core.state == 'FAULT'):
            self.probe_zero, self.probe_zero_travel = now, travel
        if self.probe_zero is not None:
            core.vx = 0.
            if core._is_stopped(now) and core.state != 'FAULT':
                if self.probe_stopped is None:
                    self.probe_stopped = now
                elif now-self.probe_stopped >= self.p.settled_sec:
                    core.state, core.reason = 'STOP', 'forward response check complete'
            else:
                self.probe_stopped = None
        else:
            core.vx = min(.15, core.vx+core.p.max_acceleration*dt)
        if core.state == 'FAULT':
            core.vx = 0.
        self.probe_result = dict(started=self.probe_started, zero_requested=self.probe_zero,
                                 zero_travel_m=self.probe_zero_travel,
                                 measured_travel_m=travel, cross_track_m=cross,
                                 stop_extra_m=travel-self.probe_zero_travel
                                 if self.probe_zero_travel is not None else None,
                                 elapsed_sec=now-self.probe_started)
        out.update(state=core.state, reason=core.reason, vx=core.vx, vy=0., wz=0.,
                   grasp_docking_complete=False, control_reference='FORWARD_RESPONSE_CHECK',
                   response_check=self.probe_result)
        return out

    def step(self, core, now, out, dt, active):
        """Keep localization, feedback, MAP association and lost-vision budgets."""
        out['vy'] = 0.0
        if self.p.forward_response_check:
            return self.response_check(core, now, out, dt)
        if core.robot is None or core.target is None:
            return out
        if self.heading is None:
            self.heading = core.search_origin[2] if core.search_origin else core.robot[2]
        report = geometry_report(core.target, core.robot, self.heading, self.p)
        completed = (core.state == 'STOP' and
                     core.reason == 'grasp docking pose reached and stopped')
        out.update(grasp_docking=report, control_reference='MAP_GRASP_DOCK',
                   grasp_docking_complete=completed,
                   distance=report['position_error_m'], docking_fixed_timeout_sec=None,
                   camera_distance_xy=math.dist(core.target, core.camera)
                   if core.camera is not None else None)
        if not active:
            self.forward.reset()
            self.stop_wait_started = self.stationary_since = None
            self.velocity_map = (0.0, 0.0)
            core.vx = core.vy = 0.0
            return out
        distance, yaw_error = report['position_error_m'], report['yaw_error_rad']
        position_ok = distance <= self.p.position_tolerance+1e-12
        yaw_ok = abs(yaw_error) <= self.p.yaw_tolerance+1e-12
        if now-core.visual_time > core.p.visual_timeout:
            if core.lost_at is None:
                core.lost_at = now
                core.odom_travel = 0.0
                core.visual_loss_count += 1
            if not core.p.allow_odom:
                core.fault('visual target lost')
            elif core.odom_travel >= core.p.odom_max_distance:
                core.fault('odometry continuation limit')
            elif max(0.0, distance-self.p.position_tolerance) > (
                    core.p.odom_max_distance-core.odom_travel):
                core.fault('remaining docking distance exceeds odometry budget')
        if distance > core.p.max_target_range:
            core.fault('docking goal out of range')
        out['odom_remaining_distance_m'] = core.p.odom_max_distance-core.odom_travel
        out['odom_remaining_time_s'] = None
        out['docking_odom_time_limit_enabled'] = False
        # Translation and yaw now happen together, so use max, not their sum.
        lower_bound = max(max(0.0, distance-self.p.position_tolerance)/core.p.max_linear_speed,
                          max(0.0, abs(yaw_error)-self.p.yaw_tolerance)/core.p.max_angular_speed)
        out['docking_motion_time_lower_bound_s'] = lower_bound

        # Progress is telemetry only. Keep correcting even if the pose stalls;
        # freshness, localization faults, odometry budgets and manual stop remain.
        error = math.hypot(max(0.0, distance-self.p.position_tolerance)/self.p.position_tolerance,
                           max(0.0, abs(yaw_error)-self.p.yaw_tolerance)/self.p.yaw_tolerance)
        if self.progress_at is None or error <= self.best_error-0.25:
            self.progress_at, self.best_error = now, error
        out.update(docking_error_metric=error, docking_last_progress_age_s=now-self.progress_at)
        out['docking_progress_timeout_enabled'] = False
        self.forward.observe(core, now)
        if core.state == 'FAULT':
            self.stop_wait_started = self.stationary_since = None
            self.velocity_map = (0.0, 0.0)
            core.vx = core.vy = 0.0
            out.update(state=core.state, reason=core.reason, vx=0.0, vy=0.0, wz=0.0)
            out['forward_step'] = self.forward.report(core)
            return out

        core.state = 'APPROACH_TO_GRASP_POSE'
        goal = report['vehicle_goal_map_pose']
        dx, dy = goal[0]-core.robot[0], goal[1]-core.robot[1]
        ex, ey = rotate_xy(-core.robot[2], (dx, dy))
        out['docking_body_error_xy'] = [ex, ey]
        # Hold a zero request across small position fluctuations while feedback
        # settles. This latch never widens the final 4cm acceptance condition.
        if self.stop_wait_started is not None:
            self.forward.stop(core, now, 'entered final stop wait')
            if (distance > self.p.settle_exit_tolerance+1e-12 or not yaw_ok or
                    (not position_ok and now-self.stop_wait_started >= self.p.settle_wait_sec)):
                self.stop_wait_started = self.stationary_since = None
        if self.stop_wait_started is None and position_ok and yaw_ok:
            self.stop_wait_started = now
        if self.stop_wait_started is not None:
            # The unchanged safety gate applies its own deceleration limit.
            self.velocity_map = (0.0, 0.0)
            core.vx = core.vy = 0.0
            if position_ok and yaw_ok and core._is_stopped(now):
                if self.stationary_since is None:
                    self.stationary_since = now
                elif now-self.stationary_since >= self.p.settled_sec:
                    core.state, core.reason = 'STOP', 'grasp docking pose reached and stopped'
            else:
                self.stationary_since = None
            out.update(state=core.state, reason=core.reason, docking_phase='STOP_WAIT',
                       vx=0.0, vy=0.0, wz=0.0,
                       docking_stop_wait_started=self.stop_wait_started,
                       docking_wait_age_s=now-self.stop_wait_started,
                       docking_pose_in_tolerance=position_ok and yaw_ok,
                       docking_stationary_since=self.stationary_since,
                       grasp_docking_complete=core.state == 'STOP')
            out['forward_step'] = self.forward.report(core)
            return out
        self.stationary_since = None
        desired = 0.0
        fine = distance <= self.p.fine_distance
        if not position_ok:
            a, latency = core.p.max_deceleration, core.p.braking_latency
            clearance = max(0.0, distance-self.p.position_tolerance/2)
            safe = math.sqrt((a*latency)**2+2*a*clearance)-a*latency
            # Minimum applies to the requested planar magnitude, before all
            # braking/slew limits; never force it on either axis separately.
            minimum = self.p.fine_min_linear_speed if fine else self.p.min_linear_speed
            maximum = self.p.fine_max_linear_speed if fine else core.p.max_linear_speed
            desired = min(core.p.max_linear_speed, maximum,
                          max(minimum, core.p.kp_linear*distance), safe)
            if core.actual_speed**2/(2*a)+core.actual_speed*latency >= clearance:
                desired = min(desired, max(0.0, core.actual_speed-a*dt))
        # Kx=Ky=kp_linear. Map-frame slew limiting also accounts for rotation of
        # the body axes between ticks; both acceleration and braking are vector bounds.
        target_map = (desired*dx/distance, desired*dy/distance) if distance else (0.0, 0.0)
        forward_override = None
        if (fine or self.forward.phase != 'IDLE') and self.p.forward_step_enabled:
            body_vx, body_vy = rotate_xy(-core.robot[2], target_map)
            forward_override = self.forward.request(
                core, now, ex, ey, body_vx, body_vy, yaw_ok)
            if forward_override is not None:
                target_map = (rotate_xy(core.robot[2], (forward_override, body_vy))
                              if forward_override > 0 else (0., 0.))
        if core.state == 'FAULT':
            self.velocity_map = (0., 0.)
            core.vx = core.vy = 0.
            out.update(state=core.state, reason=core.reason, vx=0., vy=0., wz=0.,
                       forward_step=self.forward.report(core))
            return out
        old = self.velocity_map
        delta = (target_map[0]-old[0], target_map[1]-old[1])
        decelerating = delta[0]*old[0]+delta[1]*old[1] < 0
        rate = core.p.max_deceleration if decelerating else core.p.max_acceleration
        scale = min(1.0, rate*dt/max(math.hypot(*delta), 1e-12))
        self.velocity_map = (old[0]+scale*delta[0], old[1]+scale*delta[1])
        vx, vy = rotate_xy(-core.robot[2], self.velocity_map)
        wz = max(-core.p.max_angular_speed,
                 min(core.p.max_angular_speed, core.p.kp_angular*yaw_error)) if not yaw_ok else 0.0
        if forward_override == 0.:
            wz = 0.
        core.vx, core.vy = vx, vy
        out.update(state=core.state, reason=core.reason,
                   docking_phase='FINE_CONTROL' if fine else 'CONTROL_POSE',
                   docking_stop_wait_started=None, docking_wait_age_s=0.0,
                   docking_pose_in_tolerance=position_ok and yaw_ok,
                   vx=vx, vy=vy, wz=wz, docking_stationary_since=self.stationary_since,
                   grasp_docking_complete=core.state == 'STOP')
        out['forward_step'] = self.forward.report(core)
        return out
