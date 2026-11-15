"""Per-run camera handoff protocol; imports never open a device or connect to NX."""
import json
from pathlib import Path
import time


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value)+'\n')
    temporary.replace(path)


class CameraHandoff:
    def __init__(self, directory, event=lambda *args, **kwargs: None, timeout=75.):
        self.directory = Path(directory)
        self.event = event
        self.timeout = timeout

    def request(self, action):
        atomic_json(self.directory/(action+'.request.json'), dict(action=action))
        deadline = time.monotonic()+self.timeout
        response = self.directory/(action+'.ack.json')
        while not response.exists():
            if time.monotonic() >= deadline:
                raise TimeoutError('Camera handoff timeout: '+action)
            time.sleep(.05)
        result = json.loads(response.read_text())
        self.event('CAMERA_HANDOFF_'+action.upper(), **result)
        if not result['ok']:
            raise RuntimeError('Camera handoff '+action+' failed: '+result['error'])

    def __enter__(self):
        try:
            self.request('pause')
        except BaseException:
            # Pause may have stopped the driver before a release check failed.
            self.request('resume')
            raise
        return self

    def __exit__(self, kind, value, traceback):
        self.request('resume')


def service_requests(directory, pause, resume):
    """Only the parent workflow owns mutation and acknowledges completed operations."""
    directory = Path(directory)
    for action, callback in (('pause', pause), ('resume', resume)):
        if not (directory/(action+'.request.json')).exists():
            continue
        acknowledgement = directory/(action+'.ack.json')
        if acknowledgement.exists():
            continue
        try:
            callback()
            result = dict(ok=True, action=action)
        except Exception as exc:
            result = dict(ok=False, action=action, error=str(exc))
        atomic_json(acknowledgement, result)


# Check both V4L nodes and direct libusb descriptors without enumerating/opening cameras.
RELEASE_CHECK = r'''
from pathlib import Path
import os, json, pwd

def systemd_session_helper(proc):
    # PAM and sshd session helpers retain root-owned, non-dumpable fd directories.
    # Recognize these by their role and parent; arbitrary unreadable apps still fail.
    try:
        name=(proc/'comm').read_text().strip()
        if name not in ('(sd-pam)', 'sshd'): return False
        status=dict(line.split(':',1) for line in (proc/'status').read_text().splitlines()
                    if ':' in line)
        parent=proc.parent/status['PPid'].strip()
        if name=='sshd':
            title=(proc/'cmdline').read_bytes().split(b'\0')[0].decode()
            return ((parent/'comm').read_text().strip()=='sshd' and
                    parent.stat().st_uid==0 and title.startswith(
                        'sshd: '+pwd.getpwuid(os.getuid()).pw_name+'@'))
        args=(parent/'cmdline').read_bytes().split(b'\0')
        return (parent/'comm').read_text().strip()=='systemd' and (
            Path(args[0].decode()).name=='systemd' and b'--user' in args)
    except (FileNotFoundError, PermissionError, KeyError): return False

devices=set()
for usb in Path('/sys/bus/usb/devices').iterdir():
    try:
        if (usb/'idVendor').read_text().strip() != '8086': continue
        product=(usb/'idProduct').read_text().strip()
        if product not in PRODUCTS: continue
        devices.add('/dev/bus/usb/%03d/%03d' % (
            int((usb/'busnum').read_text()), int((usb/'devnum').read_text())))
        physical=str(usb.resolve())+'/'
        for video in Path('/sys/class/video4linux').glob('video*'):
            if str(video.resolve()).startswith(physical): devices.add('/dev/'+video.name)
    except FileNotFoundError: continue
owners=[]
for proc in Path('/proc').glob('[0-9]*'):
    if int(proc.name)==os.getpid(): continue
    try:
        for descriptor in (proc/'fd').iterdir():
            try:
                target=os.readlink(str(descriptor))
                if target.endswith(' (deleted)'): target=target[:-10]
            except FileNotFoundError: continue
            if target in devices: owners.append(dict(pid=int(proc.name),device=target))
    except FileNotFoundError: continue
    except PermissionError:
        # Camera processes on this deployment run as the SSH user. An unseen process
        # of that same user must not be treated as evidence of release.
        try: same_user=proc.stat().st_uid==os.getuid()
        except FileNotFoundError: continue
        if same_user and not systemd_session_helper(proc): raise
if owners: raise RuntimeError('Camera still owned: '+json.dumps(owners))
print(json.dumps(dict(released=True, devices=sorted(devices))))
'''
