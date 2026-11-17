"""Pinned final-fusion delivery. Installation never opens serial or stops an owner."""
import base64
import hashlib
import json
import shlex
import subprocess
import zipfile
from pathlib import Path

from s1_v2_compat import compatibility_payload

PACKAGE_VERSION = '20261117_final_fusion_v4'
ARCHIVE_SHA256 = '6950d2cb8135f6ee56ccc0535dc1da6dd6a68e17478fea8cacbd5ff0bc14c90b'
ARCHIVE = 'camera_ws/s1_service/deliveries/S1_arm_final_fusion_20261117_v4.zip'
REMOTE_PACKAGE = '/home/unitree/s1-arm-control-v4'
COMPAT_VERSION = 'v4_multitarget_dump_2'


def delivery_payload(root, *, previous=False):
    path = Path(root) / ARCHIVE
    if hashlib.sha256(path.read_bytes()).hexdigest() != ARCHIVE_SHA256:
        raise RuntimeError('S1 v4 archive hash mismatch; preserve')
    with zipfile.ZipFile(path) as bundle:
        for item in bundle.infolist():
            name = Path(item.filename)
            if (not item.filename.startswith('s1-arm-control/')
                    or '..' in name.parts or item.external_attr >> 16 & 0o170000 == 0o120000):
                raise RuntimeError('Unsafe S1 archive entry: ' + item.filename)
        info = json.loads(bundle.read('s1-arm-control/BUILD_INFO.json'))
        if info['version'] != PACKAGE_VERSION:
            raise RuntimeError('Unexpected S1 v4 version')
        for line in bundle.read('s1-arm-control/SHA256SUMS').decode().splitlines():
            digest, name = line.split('  ', 1)
            if hashlib.sha256(bundle.read('s1-arm-control/' + name)).hexdigest() != digest:
                raise RuntimeError('S1 upstream manifest mismatch: ' + name)
        files = compatibility_payload(root, bundle, final_fusion=True)
    if not previous:
        name = 'scripts/run_go2_visual_alignment.py'
        source = base64.b64decode(files[name]['content'])
        old = b'record["adaptive_secure_no_lift"].setdefault("secure_force_nm", 0.40)'
        if source.count(old) != 1:
            raise RuntimeError('Unexpected S1 grasp force source; preserve')
        content = source.replace(old, old.replace(b'0.40', b'0.80'), 1)
        files[name].update(content=base64.b64encode(content).decode(),
                           sha256=hashlib.sha256(content).hexdigest())
    return path, dict(version='v4_multitarget_dump_1' if previous else COMPAT_VERSION,
                      archive_sha256=ARCHIVE_SHA256, files=files)


# Validate all upstream files, with only the source-bound adapter exceptions.
CHECK = r'''
import hashlib,json,pathlib,sys
root=pathlib.Path(sys.argv[1])
if not root.exists(): print('MISSING');raise SystemExit(0)
meta=json.loads((root/'S1_V2_COMPAT.json').read_text())
expected=json.loads(sys.stdin.read())
previous=expected.pop('__previous__',None)
upgrade=previous is not None and meta==previous
if (not upgrade and meta['version']!=sys.argv[2]) or meta['archive_sha256']!=sys.argv[3]:
    raise SystemExit('Different S1 v4 delivery; preserve')
if upgrade:expected=previous['files']
if meta['files']!=expected:raise SystemExit('S1 v4 adapter manifest mismatch')
for name,item in expected.items():
    if hashlib.sha256((root/name).read_bytes()).hexdigest()!=item['sha256']:
        raise SystemExit('Modified S1 v4 adapter; preserve: '+name)
for line in (root/'SHA256SUMS').read_text().splitlines():
    digest,name=line.split('  ',1)
    digest=expected.get(name,{}).get('sha256',digest)
    if hashlib.sha256((root/name).read_bytes()).hexdigest()!=digest:
        raise SystemExit('Modified S1 v4 upstream; preserve: '+name)
print('UPGRADE' if upgrade else 'READY')
'''

INSTALL = r'''
import base64,hashlib,json,os,pathlib,subprocess,sys,zipfile
archive=pathlib.Path(sys.argv[1]);target=pathlib.Path(sys.argv[2])
data=json.loads(sys.stdin.read())
if subprocess.run(['pgrep','-f','[r]un_arm_integration.py|[r]un_go2_left_grasp.py'],
                  stdout=subprocess.DEVNULL).returncode!=1:
    raise SystemExit('Existing S1 owner; preserve, no installation')
if hashlib.sha256(archive.read_bytes()).hexdigest()!=data['archive_sha256']:
    raise SystemExit('Uploaded S1 v4 hash mismatch')
temp=target.with_name(target.name+'.installing')
if target.exists() or temp.exists():raise SystemExit('S1 v4 destination exists; preserve')
with zipfile.ZipFile(archive) as bundle:
    for item in bundle.infolist():
        name=pathlib.PurePosixPath(item.filename)
        if (not item.filename.startswith('s1-arm-control/') or '..' in name.parts or
                item.external_attr >> 16 & 0o170000 == 0o120000):
            raise SystemExit('Unsafe S1 v4 path')
    temp.mkdir()
    for item in bundle.infolist():
        out=temp.joinpath(*pathlib.PurePosixPath(item.filename).parts[1:])
        if item.is_dir():out.mkdir(parents=True,exist_ok=True);continue
        out.parent.mkdir(parents=True,exist_ok=True)
        out.write_bytes(bundle.read(item))
for line in (temp/'SHA256SUMS').read_text().splitlines():
    digest,name=line.split('  ',1)
    if hashlib.sha256((temp/name).read_bytes()).hexdigest()!=digest:
        raise SystemExit('S1 v4 original integrity failure: '+name)
for name,item in data['files'].items():
    out=temp/name
    if out.exists():
        if hashlib.sha256(out.read_bytes()).hexdigest()!=item['original_sha256']:
            raise SystemExit('Unexpected S1 v4 original: '+name)
        backup=temp/'.v2-compat-upstream'/name
        backup.parent.mkdir(parents=True,exist_ok=True);backup.write_bytes(out.read_bytes())
    content=base64.b64decode(item.pop('content'))
    if hashlib.sha256(content).hexdigest()!=item['sha256']:
        raise SystemExit('S1 v4 adapter integrity failure: '+name)
    out.parent.mkdir(parents=True,exist_ok=True);out.write_bytes(content)
(temp/'S1_V2_COMPAT.json').write_text(json.dumps(data,indent=2)+'\n')
os.replace(temp,target)
archive.unlink()
print('S1_V4_PACKAGE_READY',target)
'''


def ensure_v4_package(root, ssh, host, *, running=False):
    path, data = delivery_payload(root)
    expected = {name: {k: v for k, v in item.items() if k != 'content'}
                for name, item in data['files'].items()}
    check_command = ['/usr/bin/python3', '-c', CHECK, REMOTE_PACKAGE,
                     COMPAT_VERSION, ARCHIVE_SHA256]
    previous = delivery_payload(root, previous=True)[1]
    previous['files'] = {name: {k: v for k, v in item.items() if k != 'content'}
                         for name, item in previous['files'].items()}
    check_input = dict(expected, __previous__=previous)
    argv = [*ssh, host, shlex.join(check_command)]
    result = subprocess.run(argv, input=json.dumps(check_input), capture_output=True,
                            text=True, timeout=30, check=True)
    if result.stdout.strip() == 'READY':
        return REMOTE_PACKAGE
    if running:
        raise RuntimeError('S1 service is running; cannot install v4 over a live owner')
    if result.stdout.strip() == 'UPGRADE':
        upgrade_command = ['/usr/bin/python3', '-c', UPGRADE, REMOTE_PACKAGE]
        subprocess.run([*ssh, host, shlex.join(upgrade_command)],
                       input=json.dumps(dict(previous=previous, current=data)),
                       text=True, timeout=30, check=True)
        subprocess.run(argv, input=json.dumps(check_input), capture_output=True,
                       text=True, timeout=30, check=True)
        return REMOTE_PACKAGE
    if result.stdout.strip() != 'MISSING':
        raise RuntimeError('Unknown S1 v4 deployment state; preserve')
    uploaded = '/tmp/go2-s1-v4-' + ARCHIVE_SHA256[:16] + '.zip'
    subprocess.run(['scp', '-i', ssh[2], '-o', 'BatchMode=yes', str(path),
                    host + ':' + uploaded], check=True, timeout=180)
    install_command = ['/usr/bin/python3', '-c', INSTALL, uploaded, REMOTE_PACKAGE]
    subprocess.run([*ssh, host, shlex.join(install_command)],
                   input=json.dumps(data), text=True, timeout=120, check=True)
    # Verify the published directory, including every calibration/plan/SDK file.
    subprocess.run(argv, input=json.dumps(check_input), capture_output=True,
                   text=True, timeout=30, check=True)
    return REMOTE_PACKAGE


UPGRADE = r'''
import base64,hashlib,json,pathlib,shutil,subprocess,sys
root=pathlib.Path(sys.argv[1]);data=json.loads(sys.stdin.read())
if subprocess.run(['pgrep','-f','[r]un_arm_integration.py|[r]un_go2_left_grasp.py'],
                  stdout=subprocess.DEVNULL).returncode!=1:
    raise SystemExit('Existing S1 owner; preserve, no upgrade')
meta=root/'S1_V2_COMPAT.json';previous=data['previous'];current=data['current']
if json.loads(meta.read_text())!=previous:raise SystemExit('S1 metadata changed; preserve')
for name,item in previous['files'].items():
    if hashlib.sha256((root/name).read_bytes()).hexdigest()!=item['sha256']:
        raise SystemExit('S1 source changed; preserve: '+name)
for line in (root/'SHA256SUMS').read_text().splitlines():
    digest,name=line.split('  ',1);digest=previous['files'].get(name,{}).get('sha256',digest)
    if hashlib.sha256((root/name).read_bytes()).hexdigest()!=digest:
        raise SystemExit('S1 upstream changed; preserve: '+name)
for name,item in current['files'].items():
    content=base64.b64decode(item.pop('content'))
    if hashlib.sha256(content).hexdigest()!=item['sha256']:
        raise SystemExit('S1 upgrade content mismatch')
    target=root/name
    if target.read_bytes()==content:continue
    backup=root/'.v2-compat-backups'/previous['files'][name]['sha256']/name
    backup.parent.mkdir(parents=True,exist_ok=True)
    if not backup.exists():shutil.copy2(target,backup)
    temp=target.with_suffix(target.suffix+'.upgrade-tmp');temp.write_bytes(content);temp.replace(target)
backup=root/'.v2-compat-backups'/previous['version']/'S1_V2_COMPAT.json'
backup.parent.mkdir(parents=True,exist_ok=True)
if not backup.exists():shutil.copy2(meta,backup)
temp=meta.with_suffix('.upgrade-tmp');temp.write_text(json.dumps(current,indent=2)+'\n');temp.replace(meta)
print('S1_V4_GRIP_FORCE_UPGRADED')
'''
