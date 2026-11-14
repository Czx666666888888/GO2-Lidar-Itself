"""Small, source-bound v2 startup/status adaptation; grasp code stays upstream."""
import base64
import hashlib
from pathlib import Path

COMPAT_VERSION = 'v2_usb01_feedback_2'


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Unexpected v2 source layout; preserve package: '+old[:70])
    return text.replace(old, new, 1)


def compatibility_payload(root, bundle):
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
    for name, content in changed.items():
        compile(content, name, 'exec')
    return {name: dict(original_sha256=hashlib.sha256(originals[name]).hexdigest()
                       if name in originals else None,
                       accepted_previous_sha256=([hashlib.sha256(previous_integration.encode()).hexdigest()]
                           if name == names[1] else []),
                       sha256=hashlib.sha256(content).hexdigest(),
                       content=base64.b64encode(content).decode())
            for name, content in changed.items()}
