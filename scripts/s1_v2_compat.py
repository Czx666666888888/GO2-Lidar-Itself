"""Small, source-bound v2 startup/status adaptation; grasp code stays upstream."""
import base64
import hashlib
from pathlib import Path

COMPAT_VERSION = 'v2_multitarget_safe_miss_6'
VIDEO3_SHA256 = {
    'scripts/run_arm_integration.py':
        '035fd4e6d73946b38e0c06ef0eaa6d3e571cb9b5b784206d7f4f0a79a06755b7',
    'demo_grasp/capture_rgbd.py':
        '8c8150066467e09bd4ae5520b0409d9559be4f0ef55e8c09f4aef10fcaf5de81',
}
RETIRED_VIDEO_FILES = {
    's1_control/grasp_video.py':
        '1638a5e7bfbd1c26639485cd233618983f9ae9276e410fa466cc6a993d79cc0c',
}


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Unexpected v2 source layout; preserve package: '+old[:70])
    return text.replace(old, new, 1)


def compatibility_payload(root, bundle, *, dual_side=True, final_fusion=False,
                          shared_grasp_version=None):
    names = ('scripts/run_arm_integration.py', 's1_control/integration.py',
             'scripts/arm_task_client.py', 'scripts/verify_handoff.py')
    originals = {name: bundle.read('s1-arm-control/'+name) for name in names}
    entry = originals[names[0]].decode()
    entry = replace_once(entry, 'from s1_control.integration import ArmIntegrationSession, ZERO_Q',
                         'from s1_control.integration import ArmIntegrationSession, ZERO_Q\n'
                         'from s1_control.startup_status import (resolve_serial_device, '
                         'serial_identity, feedback_status, communication_error)')
    entry = replace_once(entry, 'parser.add_argument("--dev", default="/dev/ttyUSB0")',
                         'parser.add_argument("--dev", default="auto")')
    entry = replace_once(entry, '    args = parser.parse_args(argv)\n',
                         '    args = parser.parse_args(argv)\n'
                         '    if args.execute: args.dev = resolve_serial_device(args.dev)\n')
    entry = replace_once(entry, '    device_lock = open(',
                         '    service_lock = open("/tmp/s1-arm-service.lock", "a")\n'
                         '    fcntl.flock(service_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)\n'
                         '    device_identity = serial_identity(args.dev)\n'
                         '    device_lock = open(')
    entry = replace_once(entry, '    session = ArmIntegrationSession(arm)\n',
                         '    session = ArmIntegrationSession(arm)\n'
                         '    session.device = args.dev\n'
                         '    session.device_identity = device_identity\n'
                         '    session._update(service_version="v2_auto_integration", '
                         'serial_mode="auto", grasp_side="left" if "left" in args.config else "right")\n')
    old = '''        arm.enable()
        fault = arm.move(q[:6], ZERO_Q, "startup_zero")
        if fault:
            raise RuntimeError(fault)
        session.start_hold()'''
    new = '''        before = feedback_status(arm, args.dev, device_identity=device_identity)
        session._update(**before)
        print("STARTUP_FEEDBACK =", json.dumps(before), flush=True)
        arm.enable()
        if not before['at_zero']:
            fault = arm.move(q[:6], ZERO_Q, "startup_zero")
            if fault:
                raise RuntimeError(fault)
            wait_for_valid_feedback(arm, arm.limits.max_temp_c)
        else:
            print("STARTUP_ZERO_SKIPPED_ALREADY_AT_ZERO", flush=True)
        after = feedback_status(arm, args.dev, device_identity=device_identity)
        if not after['at_zero']:
            raise RuntimeError("startup_zero endpoint outside original zero tolerance")
        session._update(**after)
        session.start_hold()'''
    entry = replace_once(entry, old, new)
    entry = replace_once(entry, '    finally:\n        if server is not None:',
                         '    except Exception as exc:\n'
                         '        print("S1_STARTUP_ERROR =", json.dumps(dict('
                         'readiness_state="COMMUNICATION_ERROR" if communication_error(exc) '
                         'else "FAULT", error=str(exc))), flush=True)\n'
                         '        raise\n    finally:\n        if server is not None:')
    integration = originals[names[1]].decode()
    if final_fusion:
        # Keep dump on the original generic queue; latch side/miss only for grasp.
        integration = replace_once(integration,
            '    def request_grasp(self):\n        return self.request_task("grasp")',
            '    def request_grasp(self):\n'
            '        with self._lock:\n'
            '            if self._state["state"] != "ZERO_HOLD":\n'
            '                return {"accepted": False, "reason": self._state["state"]}\n'
            '            self._state.update(state="QUEUED", busy=True, navigation_allowed=False)\n'
            '        self.jobs.put("grasp")\n'
            '        return {"accepted": True, "state": "QUEUED"}')
        integration = replace_once(integration,
            '            self._state.update(state="QUEUED", busy=True, navigation_allowed=False)\n'
            '        self.jobs.put("grasp" if command == "grasp" else (command, payload))',
            '            self._state.update(**dict(state="QUEUED", busy=True, navigation_allowed=False))\n'
            '        self.jobs.put("grasp" if command == "grasp" else (command, payload))')
    integration = replace_once(integration, 'import time\n',
                               'import time\nfrom .startup_status import feedback_status, communication_error\n')
    integration = replace_once(integration, '    def snapshot(self):\n        with self._lock:',
                               '''    def snapshot(self):
        try:
            values = feedback_status(self.arm, getattr(self, 'device', None),
                                     device_identity=getattr(self, 'device_identity', None))
            with self._lock:
                state = self._state['state']
                values['readiness_state'] = ('BUSY' if self._state['busy'] else
                    'ZERO_HOLD' if state == 'ZERO_HOLD' and values['at_zero'] else
                    'FAULT' if state.startswith('FAULT') else values['position_state'])
                values['navigation_allowed'] = (state == 'ZERO_HOLD'
                    and not self._state['busy'] and values['at_zero'] and self._fault is None)
                self._state.update(values)
        except Exception as exc:
            self._update(readiness_state='COMMUNICATION_ERROR' if communication_error(exc)
                         else 'FAULT', error=str(exc), navigation_allowed=False)
        with self._lock:''')
    previous_integration = replace_once(integration,
        """                values['navigation_allowed'] = (state == 'ZERO_HOLD'
                    and not self._state['busy'] and values['at_zero'] and self._fault is None)""",
        """                if state == 'ZERO_HOLD' and not values['at_zero']:
                    values['navigation_allowed'] = False""")
    client = replace_once(originals[names[2]].decode(),
                          '        client.connect(socket_path)',
                          '        client.settimeout(3.0)\n        client.connect(socket_path)')
    changed = dict(zip(names, (entry.encode(), integration.encode(), client.encode())))
    verify = originals[names[3]].decode()
    verify = replace_once(verify, '    for line in (ROOT/"SHA256SUMS").read_text().splitlines():',
                           '    compat_path=ROOT/"S1_V2_COMPAT.json"\n'
                           '    compat=json.loads(compat_path.read_text())["files"] if '
                           'compat_path.exists() else {}\n'
                           '    for name,meta in compat.items():\n'
                           '        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=meta["sha256"]:\n'
                           '            raise RuntimeError("compatibility source changed: "+name)\n'
                           '    for line in (ROOT/"SHA256SUMS").read_text().splitlines():')
    verify = replace_once(verify, '        digest,relative=line.split("  ",1)',
                           '        digest,relative=line.split("  ",1)\n'
                           '        digest=compat.get(relative,{}).get("sha256",digest)')
    changed[names[3]] = verify.encode()
    helper = 's1_control/startup_status.py'
    changed[helper] = (Path(root)/'camera_ws/s1_service'/helper).read_bytes()
    # Restore original camera acquisition and direct grasp callback after video_3.
    capture_name = 'demo_grasp/capture_rgbd.py'
    originals[capture_name] = bundle.read('s1-arm-control/'+capture_name)
    changed[capture_name] = originals[capture_name]
    previous4 = dict(changed)
    if dual_side:
        helper = """def selected_grasp_arguments(args, side):
    if side not in ('right', 'left'):
        raise ValueError('Unknown grasp side')
    selected = argparse.Namespace(**vars(args))
    if side == ('left' if 'left' in args.config else 'right'):
        return selected
    selected.config = ('config/go2_left_full_flow_size_adaptive_20261112_v1.json'
        if side == 'left' else 'config/go2_dual_white_light_full_flow_size_adaptive_v1.json')
    selected.search_plan = (['data/plans/go2_left_grasp_search_20261112_v1.json',
                            'data/plans/go2_left_grasp_search_20261112_v1_report.json']
        if side == 'left' else ['data/plans/go2_right_grasp_search_20261107_v1.json',
                               'data/plans/go2_right_grasp_search_20261107_v1_report.json'])
    return selected


"""
        entry = replace_once(entry, 'def main(argv=None):', helper+'def main(argv=None):')
        entry = replace_once(entry, 'response = session.request_grasp()',
            'response = session.request_grasp(request.get("grasp_side", '
            '"left" if "left" in args.config else "right"))')
        entry = replace_once(entry, 'serial_mode="auto", grasp_side=',
                             'serial_mode="auto", grasp_sides_supported=["right", "left"], grasp_side=')
        indent = '                ' if final_fusion else '            '
        entry = replace_once(entry,
            indent+'session.run_task(lambda active: flow.main(flow_arguments(args, execute=True),',
            indent+'task_args = selected_grasp_arguments(args, session.snapshot()["grasp_side"])\n'+
            indent+'session.run_task(lambda active: flow.main(flow_arguments(task_args, execute=True),')
        integration = replace_once(integration, '    def request_grasp(self):',
            '    def request_grasp(self, side=None):\n'
            '        if side is not None and side not in ("right", "left"):\n'
            '            return dict(accepted=False, reason="UNKNOWN_GRASP_SIDE")')
        integration = replace_once(integration,
            '            self._state.update(state="QUEUED", busy=True, navigation_allowed=False)',
            '            if side is not None: self._state["grasp_side"] = side\n'
            '            self._state.update(state="QUEUED", busy=True, navigation_allowed=False)')
        client = replace_once(client, 'def request(command, socket_path=DEFAULT_SOCKET):',
                             'def request(command, socket_path=DEFAULT_SOCKET, grasp_side=None):')
        client = replace_once(client, '        client.sendall((json.dumps({"command": command})+"\\n").encode())',
            '        payload = {"command": command}\n'
            '        if grasp_side is not None: payload["grasp_side"] = grasp_side\n'
            '        client.sendall((json.dumps(payload)+"\\n").encode())')
        client = replace_once(client, '    args = parser.parse_args()',
            '    parser.add_argument("--grasp-side", choices=["right", "left"])\n'
            '    args = parser.parse_args()')
        client = replace_once(client, 'request(args.command, args.socket)',
                             'request(args.command, args.socket, args.grasp_side)')
        changed.update({names[0]: entry.encode(), names[1]: integration.encode(),
                        names[2]: client.encode()})
    if dual_side:
        previous5 = dict(changed)
        if shared_grasp_version is None:
            flow_name = 'scripts/run_go2_visual_alignment.py'
            originals[flow_name] = bundle.read('s1-arm-control/'+flow_name)
            flow = originals[flow_name].decode()
            flow = replace_once(flow, 'def main(argv=None, *, arm_session=None) -> int:',
                'class MemoryTargetMiss(RuntimeError):\n    pass\n\ndef main(argv=None, *, arm_session=None) -> int:')
            flow = replace_once(flow, '        observation_record = None\n        prepared_alignment = None',
                '        if (getattr(arm_session, "allow_missed", False) and\n'
                '                initial_capture["detection"]["status"] == "NO_TARGET" and\n'
                '                initial_capture["detection"].get("plane", {}).get("quality_pass") is True):\n'
                '            raise MemoryTargetMiss("D455 valid capture has no target")\n'
                '        observation_record = None\n        prepared_alignment = None')
            flow = replace_once(flow, '                print("SAFE_RETURN_AFTER_REJECT = True")',
                '                if isinstance(exc, MemoryTargetMiss):\n'
                '                    result = "VISUAL_NO_TARGET_SAFE_RETURN"\n'
                '                print("SAFE_RETURN_AFTER_REJECT = True")')
            # The exception recovery already performs collision-checked reverse deployment.
            # Only its proved empty-target return receives a nonfault terminal code.
            flow = replace_once(flow, '        return 1\n    finally:\n        if arm is not None:',
                '        return 3 if result == "VISUAL_NO_TARGET_SAFE_RETURN" and not deployed else 1\n'
                '    finally:\n        if arm is not None:')
            changed[flow_name] = flow.encode()
        entry = changed[names[0]].decode()
        entry = replace_once(entry, 'response = session.request_grasp(request.get("grasp_side", '
            '"left" if "left" in args.config else "right"))',
            'response = session.request_grasp(request.get("grasp_side", '
            '"left" if "left" in args.config else "right"), allow_missed=request.get("allow_missed", False))')
        integration = changed[names[1]].decode()
        integration = replace_once(integration, 'def request_grasp(self, side=None):',
                                   'def request_grasp(self, side=None, allow_missed=False):')
        integration = replace_once(integration,
            '            if side is not None: self._state["grasp_side"] = side',
            '            self.allow_missed = allow_missed is True\n'
            '            if side is not None: self._state["grasp_side"] = side')
        client = changed[names[2]].decode()
        client = replace_once(client, 'def request(command, socket_path=DEFAULT_SOCKET, grasp_side=None):',
            'def request(command, socket_path=DEFAULT_SOCKET, grasp_side=None, allow_missed=False):')
        client = replace_once(client, '        payload = {"command": command}',
            '        payload = {"command": command}\n'
            '        if allow_missed: payload["allow_missed"] = True')
        changed.update({names[0]: entry.encode(), names[1]: integration.encode(), names[2]: client.encode()})
    if final_fusion:
        # Both tasks must reject a stale ZERO_HOLD after a serial/feedback fault.
        integration = changed[names[1]].decode()
        for signature in ('    def request_grasp(self, side=None, allow_missed=False):',
                          '    def request_task(self, command, payload=None):'):
            integration = replace_once(integration, signature,
                signature+'\n'
                '        if self.snapshot().get("readiness_state") != "ZERO_HOLD":\n'
                '            return dict(accepted=False, reason="S1_NOT_READY")')
        changed[names[1]] = integration.encode()
        changed[names[0]] = changed[names[0]].replace(
            b'service_version="v2_auto_integration"',
            b'service_version="' + (shared_grasp_version or '20261117_final_fusion_v4').encode()
            + b'", task_commands_supported=["status", "grasp", "dump"]')
    for name, content in changed.items():
        compile(content, name, 'exec')
    return {name: dict(original_sha256=hashlib.sha256(originals[name]).hexdigest()
                       if name in originals else None,
                       accepted_previous_sha256=(
                           [hashlib.sha256(previous_integration.encode()).hexdigest()]
                           if name == names[1] else []) + (
                           [VIDEO3_SHA256[name]] if name in VIDEO3_SHA256 else []) + (
                           [hashlib.sha256(previous4[name]).hexdigest()]
                           if dual_side and name in previous4 and previous4[name] != content else []) + (
                           [hashlib.sha256(previous5[name]).hexdigest()]
                           if dual_side and name in previous5 and previous5[name] != content else []),
                       sha256=hashlib.sha256(content).hexdigest(),
                       content=base64.b64encode(content).decode())
            for name, content in changed.items()}
