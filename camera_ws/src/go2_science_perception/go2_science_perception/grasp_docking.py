"""Small optional docking stage after the stable MAP target confirmation."""
import math

from .grasp_pose_geometry import DockSettings, geometry_report, wrap


class GraspDocking:
    def __init__(self, settings=None):
        self.p = settings or DockSettings()
        self.reset()

    def reset(self):
        self.heading = None
        self.stationary_since = None
        self.last_wz = 0.0
        self.final_alignment = False
        self.braked_for_alignment = False

    def step(self, core, now, out, dt, active):
        """Reuse core input validity, MAP anchor, speed limits and loss protection."""
        if core.robot is None or core.target is None:
            return out
        if self.heading is None:
            self.heading = core.search_origin[2] if core.search_origin else core.robot[2]
        report = geometry_report(core.target, core.robot, self.heading, self.p)
        completed = (core.state == 'STOP' and
                     core.reason == 'grasp docking pose reached and stopped')
        out.update(grasp_docking=report, control_reference='MAP_GRASP_DOCK',
                   grasp_docking_complete=completed,
                   distance=report['position_error_m'],
                   camera_distance_xy=math.dist(core.target, core.camera)
                   if core.camera is not None else None)
        if not active:
            return out
        if now-core.visual_time > core.p.visual_timeout:
            if core.lost_at is None:
                core.lost_at = now
                core.odom_travel = 0.0
                core.visual_loss_count += 1
            if not core.p.allow_odom:
                core.fault('visual target lost')
            elif (now-core.lost_at >= core.p.odom_max_time or
                  core.odom_travel >= core.p.odom_max_distance):
                core.fault('odometry continuation limit')
            elif report['position_error_m'] > core.p.odom_max_distance-core.odom_travel:
                core.fault('remaining docking distance exceeds odometry budget')
        goal = report['vehicle_goal_map_pose']
        dx, dy = goal[0]-core.robot[0], goal[1]-core.robot[1]
        distance = report['position_error_m']
        if distance > core.p.max_target_range:
            core.fault('docking goal out of range')
        out['odom_remaining_distance_m'] = core.p.odom_max_distance-core.odom_travel
        out['odom_remaining_time_s'] = (core.p.odom_max_time-(now-core.lost_at)
                                        if core.lost_at is not None else core.p.odom_max_time)
        # A lower bound, not a prediction of actual travel/settling time.
        yaw_to_position = wrap(math.atan2(dy, dx)-core.robot[2])
        lower_bound = distance/core.p.max_linear_speed
        lower_bound += abs(report['yaw_error_rad'])/core.p.max_angular_speed
        out['docking_motion_time_lower_bound_s'] = lower_bound
        if core.lost_at is not None and lower_bound > out['odom_remaining_time_s']:
            core.fault('minimum docking time exceeds odometry budget')
        if core.state == 'FAULT':
            out.update(state=core.state, reason=core.reason, vx=0.0, wz=0.0)
            return out
        core.state = 'APPROACH_TO_GRASP_POSE'
        position_ok = distance <= self.p.position_tolerance
        yaw_ok = abs(report['yaw_error_rad']) <= self.p.yaw_tolerance
        # Once inside the position tolerance, brake before rotating. Keep the
        # final-heading objective through position jitter; still require the
        # ORIGINAL position tolerance and S1 region before declaring completion.
        if position_ok and not self.final_alignment:
            self.final_alignment = True
            self.braked_for_alignment = False
        if self.final_alignment and not self.braked_for_alignment and core._is_stopped(now):
            self.braked_for_alignment = True
        if self.final_alignment and self.braked_for_alignment and yaw_ok and not position_ok:
            self.final_alignment = False
        vx, wz = 0.0, 0.0
        if self.final_alignment and not self.braked_for_alignment:
            self.stationary_since = None
            phase = 'BRAKE_FOR_FINAL_YAW'
        elif self.final_alignment and not yaw_ok:
            self.stationary_since = None
            phase = 'FINAL_YAW'
            wz = core.p.kp_angular*report['yaw_error_rad']
        elif not position_ok:
            self.stationary_since = None
            error = yaw_to_position
            phase = 'TURN_TO_POSITION' if abs(error) > core.p.align_yaw_threshold else 'TRANSLATE'
            if phase == 'TRANSLATE':
                a, latency = core.p.max_deceleration, core.p.braking_latency
                clearance = max(0.0, distance-self.p.position_tolerance/2)
                safe = math.sqrt((a*latency)**2+2*a*clearance)-a*latency
                desired = min(core.p.max_linear_speed, core.p.kp_linear*distance, safe)
                brake = core.actual_speed**2/(2*a)+core.actual_speed*latency
                if brake >= clearance:
                    desired = min(desired, max(0.0, core.actual_speed-a*dt))
                vx = min(desired, core.vx+core.p.max_acceleration*dt)
            wz = core.p.kp_angular*error
        elif not report['predicted_in_grasp_region']:
            core.fault('dock pose tolerance reached but target outside S1 region')
            phase = 'REJECT_REGION'
        else:
            phase = 'SETTLE'
            if core._is_stopped(now):
                if self.stationary_since is None:
                    self.stationary_since = now
                elif now-self.stationary_since >= self.p.settled_sec:
                    core.state, core.reason = 'STOP', 'grasp docking pose reached and stopped'
            else:
                self.stationary_since = None
        core.vx = vx if core.state not in ('FAULT', 'STOP') else 0.0
        self.last_wz = max(-core.p.max_angular_speed, min(core.p.max_angular_speed, wz))
        out.update(state=core.state, reason=core.reason, docking_phase=phase,
                   vx=core.vx, wz=self.last_wz if core.state not in ('FAULT', 'STOP') else 0.0,
                   docking_stationary_since=self.stationary_since,
                   grasp_docking_complete=core.state == 'STOP')
        return out
