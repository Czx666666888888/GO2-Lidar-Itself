#!/usr/bin/env python3
"""Persistent zero-hold service. No hardware is opened without --execute."""
from __future__ import annotations
import argparse
import fcntl
import json
from pathlib import Path
import queue
import signal
import socketserver
import sys
import threading
import os
import time

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / "scripts", Path.home()/"thirdparty/S1_SDK_V2-official/src"):
    sys.path.insert(0, str(path))
from s1_control.integration import ArmIntegrationSession, ZERO_Q
from s1_control.startup_status import (
    feedback_status, communication_error, resolve_serial_device, serial_identity,
)

DEFAULT_SOCKET = "/tmp/s1-arm-integration.sock"

def flow_arguments(args, *, execute):
    values = ["--dev", args.dev, "--config", args.config,
              "--search-plan", "data/plans/go2_right_grasp_search_20261107_v1.json",
              "data/plans/go2_right_grasp_search_20261107_v1_report.json",
              "--flexible-approach", "--all-joint-speed-rad-s", str(args.speed),
              "--descent-speed-rad-s", str(args.descent_speed), "--non-interactive"]
    if execute:
        values.append("--execute")
    return values

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dev", default="auto", help="auto-select the unique ttyUSB0 or ttyUSB1")
    parser.add_argument("--socket", default=DEFAULT_SOCKET)
    parser.add_argument("--config", default="config/go2_dual_white_light_full_flow_size_adaptive_v1.json")
    parser.add_argument("--speed", type=float, default=1.5)
    parser.add_argument("--descent-speed", type=float, default=0.75)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--status-file", default="/tmp/s1-arm-startup-status.json")
    args = parser.parse_args(argv)
    if args.execute:
        args.dev = resolve_serial_device(args.dev)
    def startup_report(**values):
        values["host_time_ns"] = time.time_ns()
        target = Path(args.status_file)
        temporary = target.with_suffix(".tmp")
        temporary.write_text(json.dumps(values)+"\n")
        os.replace(temporary, target)
        print("S1_STARTUP " + json.dumps(values), flush=True)
    import run_go2_visual_alignment as flow
    # Run the complete existing preflight before opening the arm.
    code = flow.main(flow_arguments(args, execute=False))
    if code or not args.execute:
        if code:
            startup_report(state='FAULT', busy=False, error='original S1 preflight failed: '+str(code))
        return code
    from s1_control.arm import Arm, SafetyLimits
    from execute_go2_observe_deployment import wait_for_valid_feedback
    from s1_control.go2_deployment import start_window_violations
    _, config = flow.load_alignment_config(args.config)
    plan = json.loads(flow.resolve_project_path(config["deployment_plan"]).read_text())
    # One service owns the device across idle hold and every grasp.
    service_lock = open('/tmp/s1-arm-service.lock', 'a')
    fcntl.flock(service_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    device_lock = open("/tmp/s1-arm-" + Path(args.dev).resolve().name + ".lock", "a")
    fcntl.flock(device_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    socket_path = Path(args.socket)
    if socket_path.is_socket():
        socket_path.unlink()
    startup_report(state='STARTING', busy=True)
    try:
        expected_identity = serial_identity(args.dev)
        arm = Arm(dev=args.dev, end_effector="gripper", check_collision=False,
                  limits=SafetyLimits(v_max_rad_s=args.speed, j6_v_max_rad_s=args.speed,
                                      max_speed_rad_s=1.2*args.speed, motion_profile="s_curve",
                                      pause_on_tracking_lag=True, elapsed_cycle_progress=True))
        if serial_identity(args.dev) != expected_identity:
            raise OSError('serial device re-enumerated while opening: '+args.dev)
    except Exception as exc:
        startup_report(state='COMMUNICATION_ERROR' if communication_error(exc) else 'FAULT',
                       busy=False, error=str(exc))
        raise
    session = ArmIntegrationSession(arm)
    session.device = args.dev
    session.device_identity = expected_identity
    session.zero_windows = plan["start_safe_windows_rad"]
    stop = threading.Event()
    server = None
    server_running = False
    def stop_requested(signum, frame):
        # Finish an active grasp before stopping the service.
        stop.set()
    signal.signal(signal.SIGTERM, stop_requested)
    signal.signal(signal.SIGINT, stop_requested)
    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            try:
                request = json.loads(self.rfile.readline(4096))
                if request.get("command") == "status":
                    response = session.snapshot()
                elif request.get("command") == "zero":
                    response = session.request_zero()
                elif request.get("command") == "grasp":
                    response = session.request_grasp()
                else:
                    response = {"accepted": False, "reason": "unknown command"}
            except (ValueError, TypeError) as exc:
                response = {"accepted": False, "reason": str(exc)}
            self.wfile.write((json.dumps(response, ensure_ascii=False)+"\n").encode())
    class Server(socketserver.ThreadingUnixStreamServer):
        daemon_threads = True
    try:
        def ensure_zero():
            q, _, _ = wait_for_valid_feedback(arm, arm.limits.max_temp_c)
            status = feedback_status(arm, args.dev, session.zero_windows, session.device_identity)
            violations = start_window_violations(q, plan['start_safe_windows_rad'])
            session._update(**status, navigation_allowed=False)
            startup_report(state=status['position_state'], busy=False, **status)
            if violations:
                # Original verified start window is a path constraint, not a bus fault.
                session._update(state='NOT_AT_ZERO', readiness_state='NOT_AT_ZERO', busy=False,
                                return_zero_allowed=False, error='unverified return path: '+str(violations))
                startup_report(**session.snapshot())
                return False
            session.stop_hold()
            session._update(state='RETURNING_ZERO', busy=True, return_zero_allowed=True)
            arm.enable()
            if not status['at_zero']:
                fault = arm.move(q[:6], ZERO_Q, 'startup_zero')
                if fault:
                    raise RuntimeError(fault)
            # Verify the real endpoint before commanding the existing zero hold.
            q, _, _ = wait_for_valid_feedback(arm, arm.limits.max_temp_c)
            status = feedback_status(arm, args.dev, session.zero_windows, session.device_identity)
            if not status['at_zero']:
                raise RuntimeError('controlled zero finished outside original zero tolerance: '+str(q))
            session.start_hold()
            session.ready()
            startup_report(**session.snapshot())
            return True
        server = Server(args.socket, Handler)
        Path(args.socket).chmod(0o660)
        ensure_zero()
        threading.Thread(target=server.serve_forever, name="s1-task-api", daemon=True).start()
        server_running = True
        print("INTEGRATION_READY = "+session.snapshot()["state"]+" socket="+args.socket, flush=True)
        while not stop.is_set():
            try:
                job = session.jobs.get(timeout=0.2)
            except queue.Empty:
                continue
            if job == 'zero':
                ensure_zero()
                continue
            session.run_task(lambda active: flow.main(flow_arguments(args, execute=True),
                                                     arm_session=active))
        return 0
    except Exception as exc:
        startup_report(state='COMMUNICATION_ERROR' if communication_error(exc) else 'FAULT',
                       busy=False, error=str(exc))
        raise
    finally:
        if server is not None:
            if server_running:
                server.shutdown()
            server.server_close()
            Path(args.socket).unlink(missing_ok=True)
        session.close()

if __name__ == "__main__":
    raise SystemExit(main())
