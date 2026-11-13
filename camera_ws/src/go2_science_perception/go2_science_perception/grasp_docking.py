"""Optional body-frame planar docking through the existing velocity safety gate."""
import math

from .grasp_pose_geometry import DockSettings, geometry_report, rotate_xy


class GraspDocking:
    def __init__(self, settings=None):
        self.p = settings or DockSettings()
        self.reset()

    def reset(self):
        self.heading = None
        self.stationary_since = None
        self.velocity_map = (0.0, 0.0)
        self.progress_at = None
        self.best_error = math.inf

    def step(self, core, now, out, dt, active):
        """Keep localization, feedback, MAP association and lost-vision budgets."""
        out['vy'] = 0.0
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
        if core.state == 'FAULT':
            self.velocity_map = (0.0, 0.0)
            core.vx = core.vy = 0.0
            out.update(state=core.state, reason=core.reason, vx=0.0, vy=0.0, wz=0.0)
            return out

        core.state = 'APPROACH_TO_GRASP_POSE'
        goal = report['vehicle_goal_map_pose']
        dx, dy = goal[0]-core.robot[0], goal[1]-core.robot[1]
        ex, ey = rotate_xy(-core.robot[2], (dx, dy))
        out['docking_body_error_xy'] = [ex, ey]
        desired = 0.0
        if not position_ok:
            a, latency = core.p.max_deceleration, core.p.braking_latency
            clearance = max(0.0, distance-self.p.position_tolerance/2)
            safe = math.sqrt((a*latency)**2+2*a*clearance)-a*latency
            # Minimum applies to the requested planar magnitude, before all
            # braking/slew limits; never force it on either axis separately.
            desired = min(core.p.max_linear_speed,
                          max(self.p.min_linear_speed, core.p.kp_linear*distance), safe)
            if core.actual_speed**2/(2*a)+core.actual_speed*latency >= clearance:
                desired = min(desired, max(0.0, core.actual_speed-a*dt))
        # Kx=Ky=kp_linear. Map-frame slew limiting also accounts for rotation of
        # the body axes between ticks; both acceleration and braking are vector bounds.
        target_map = (desired*dx/distance, desired*dy/distance) if distance else (0.0, 0.0)
        old = self.velocity_map
        delta = (target_map[0]-old[0], target_map[1]-old[1])
        decelerating = delta[0]*old[0]+delta[1]*old[1] < 0
        rate = core.p.max_deceleration if decelerating else core.p.max_acceleration
        scale = min(1.0, rate*dt/max(math.hypot(*delta), 1e-12))
        self.velocity_map = (old[0]+scale*delta[0], old[1]+scale*delta[1])
        vx, vy = rotate_xy(-core.robot[2], self.velocity_map)
        wz = max(-core.p.max_angular_speed,
                 min(core.p.max_angular_speed, core.p.kp_angular*yaw_error)) if not yaw_ok else 0.0
        phase = 'CONTROL_POSE'
        if position_ok and yaw_ok:
            phase = 'SETTLE'
            if core._is_stopped(now):
                if self.stationary_since is None:
                    self.stationary_since = now
                elif now-self.stationary_since >= self.p.settled_sec:
                    core.state, core.reason = 'STOP', 'grasp docking pose reached and stopped'
            else:
                self.stationary_since = None
        else:
            self.stationary_since = None
        if core.state == 'STOP':
            self.velocity_map = (0.0, 0.0)
            vx = vy = wz = 0.0
        core.vx, core.vy = vx, vy
        out.update(state=core.state, reason=core.reason, docking_phase=phase,
                   vx=vx, vy=vy, wz=wz, docking_stationary_since=self.stationary_since,
                   grasp_docking_complete=core.state == 'STOP')
        return out
