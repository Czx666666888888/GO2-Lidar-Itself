#!/usr/bin/env python3
"""GO2-host posture adapter for the existing arm socket. No action without --execute."""
import argparse
import importlib.util
import json
import math
import time
import sys
import threading


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--require-heartbeat', action='store_true',
                   help='PC full-task supervisor must keep stdin heartbeats alive')
    p.add_argument('--arm-client', required=True)
    p.add_argument('--socket', default='/tmp/s1-arm-integration.sock')
    p.add_argument('--interface', default='eth0')
    p.add_argument('--state-topic', default='rt/lf/sportmodestate')
    p.add_argument('--down-max-body-height-m', type=float, required=True)
    p.add_argument('--up-min-body-height-m', type=float, required=True)
    p.add_argument('--down-mode', type=int, required=True)
    p.add_argument('--up-mode', type=int, required=True)
    p.add_argument('--posture-timeout', type=float, default=20.0)
    p.add_argument('--stable-sec', type=float, default=1.0)
    p.add_argument('--arm-timeout', type=float, default=300.0)
    args = p.parse_args()
    if not (0 < args.down_max_body_height_m < args.up_min_body_height_m and
            args.posture_timeout > 0 and args.stable_sec > 0 and args.arm_timeout > 0):
        p.error('Use measured separate posture boundaries and positive time limits')
    if not args.execute:
        print(json.dumps(dict(execute=False, plan=['StandDown', 'grasp', 'ZERO_HOLD', 'StandUp'],
                              arm_client=args.arm_client, s1_base_z=None)))
        return
    last_heartbeat = [time.monotonic()]
    cancelled = threading.Event()

    def supervisor_input():
        try:
            for line in sys.stdin:
                message = json.loads(line)
                if message.get('stop'):
                    break
                if message.get('heartbeat'):
                    last_heartbeat[0] = time.monotonic()
        finally:
            cancelled.set()

    if args.require_heartbeat:
        threading.Thread(target=supervisor_input, daemon=True).start()

    def check_supervisor():
        expired = cancelled.is_set() or time.monotonic()-last_heartbeat[0] > 3.0
        if args.require_heartbeat and expired:
            raise RuntimeError('PC stop/heartbeat lost; no subsequent posture or arm action')

    # Read/import ONLY the supplied stdlib client; never import/open an arm SDK or serial device.
    spec = importlib.util.spec_from_file_location('existing_arm_client', args.arm_client)
    client = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(client)
    # Bound the existing client's sockets without changing global SDK socket defaults.
    from types import SimpleNamespace
    original_socket = client.socket

    def timed_socket(*values):
        connection = original_socket.socket(*values)
        connection.settimeout(2.0)
        return connection

    client.socket = SimpleNamespace(socket=timed_socket, AF_UNIX=original_socket.AF_UNIX,
                                    SOCK_STREAM=original_socket.SOCK_STREAM)
    try:
        from go2_science_perception.s1_task_sequence import finish_docked_task, require_zero_hold
    except ImportError:
        # Deploy this pure module beside this host adapter when GO2 uses Foxy/Python3.8.
        from s1_task_sequence import finish_docked_task, require_zero_hold
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
    from unitree_sdk2py.go2.sport.sport_client import SportClient

    def log(event, data):
        print(json.dumps(dict(event=event, host_time_ns=time.time_ns(), data=data),
                         allow_nan=False), flush=True)

    require_zero_hold(client.request('status', args.socket))
    ChannelFactoryInitialize(0, args.interface)
    last = [None, -math.inf]

    def on_state(msg):
        last[:] = [msg, time.monotonic()]

    subscriber = ChannelSubscriber(args.state_topic, SportModeState_)
    subscriber.Init(on_state, 10)
    # Exclusive SDK motion ownership begins only AFTER the PC controller/gate were shut down.
    sdk = SportClient(enableLease=True)
    sdk.SetTimeout(3.0)
    sdk.Init()
    deadline = time.monotonic()+args.posture_timeout
    while not sdk.GetLeaseId():
        check_supervisor()
        if time.monotonic() >= deadline:
            raise RuntimeError('SDK Sport lease acquisition timeout; no posture call')
        time.sleep(0.1)
    holding_down = [False]

    def feedback():
        check_supervisor()
        msg, received = last
        if msg is None or time.monotonic()-received > 0.5:
            raise RuntimeError('Fresh GO2 state missing')
        values = [float(msg.body_height), *[float(x) for x in msg.velocity],
                  float(msg.yaw_speed), *[float(x) for x in msg.imu_state.gyroscope]]
        if not all(math.isfinite(x) for x in values):
            raise RuntimeError('Nonfinite GO2 state')
        return dict(body_height_m=values[0], mode=int(msg.mode), error_code=int(msg.error_code),
                    velocity_xy_mps=math.hypot(values[1], values[2]),
                    yaw_rate=float(msg.yaw_speed),
                    angular_rate_max=max(abs(float(x)) for x in msg.imu_state.gyroscope),
                    received_age_s=time.monotonic()-received)

    def request(command):
        state = feedback()
        if holding_down[0] and (state['body_height_m'] > args.down_max_body_height_m or
                                state['mode'] != args.down_mode or
                                state['velocity_xy_mps'] > 0.05 or
                                abs(state['yaw_rate']) > 0.10):
            raise RuntimeError('GO2 down/quiet condition lost during arm task; no StandUp')
        return client.request(command, args.socket)

    def posture(direction):
        require_zero_hold(request('status'))
        initial = feedback()
        if initial['velocity_xy_mps'] > 0.05 or abs(initial['yaw_rate']) > 0.10:
            raise RuntimeError('GO2 is not stopped before posture handoff')
        code = sdk.StopMove()
        if code != 0:
            raise RuntimeError('StopMove RPC failed: '+str(code))
        holding_down[0] = False
        code = sdk.StandDown() if direction == 'down' else sdk.StandUp()
        log('POSTURE_RPC', dict(direction=direction, code=code))
        if code != 0:
            raise RuntimeError('Posture RPC failed; no grasp/automatic recovery')
        deadline = time.monotonic()+args.posture_timeout
        stable_since = None
        heights = []
        while time.monotonic() < deadline:
            require_zero_hold(client.request('status', args.socket))
            state = feedback()
            height_ok = (state['body_height_m'] <= args.down_max_body_height_m
                         if direction == 'down' else
                         state['body_height_m'] >= args.up_min_body_height_m)
            mode_ok = state['mode'] == (args.down_mode if direction == 'down' else args.up_mode)
            quiet = (state['velocity_xy_mps'] <= 0.05 and abs(state['yaw_rate']) <= 0.10 and
                     state['angular_rate_max'] <= 0.10)
            now = time.monotonic()
            if height_ok and mode_ok and quiet:
                if stable_since is None:
                    stable_since = now
                    heights = []
                heights.append(state['body_height_m'])
                if max(heights)-min(heights) > 0.01:
                    stable_since = None
                elif now-stable_since >= args.stable_sec:
                    holding_down[0] = direction == 'down'
                    result = dict(confirmed=True, feedback=state,
                                  criteria='measured height/mode + continuous quiet feedback',
                                  physical_belly_contact_independently_verified=None)
                    log('POSTURE_FEEDBACK_CONFIRMED', result)
                    return result
            else:
                stable_since = None
            time.sleep(0.05)
        raise RuntimeError('Posture feedback timeout; no subsequent action')

    result = finish_docked_task(request, posture, log, timeout=args.arm_timeout)
    log('RESULT', result)


if __name__ == '__main__':
    main()
