#!/usr/bin/env python3
"""Deploy the user's S1 v2 integration ZIP as a separate, versioned host package.

No arm device is opened here. A running serial owner is never replaced.
"""
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import zipfile
from s1_v2_compat import COMPAT_VERSION, compatibility_payload

PACKAGE_VERSION = 'v2_auto_integration'
DEFAULT_ARCHIVE = 'S1_arm_integration_20261114_auto_v2.zip'
REMOTE_PACKAGE = '/home/unitree/s1-arm-control-v2'


def ensure_v2_compat(root, ssh, host, *, running=False, archive=None):
    check = r'''
import hashlib,json,pathlib,sys
root=pathlib.Path(sys.argv[1]);p=root/'S1_V2_COMPAT.json'
if not p.exists(): print('MISSING');raise SystemExit(0)
d=json.loads(p.read_text())
if d['version']!=sys.argv[2]: raise SystemExit('Unknown S1 v2 compatibility version; preserve')
for name,meta in d['files'].items():
    if hashlib.sha256((root/name).read_bytes()).hexdigest()!=meta['sha256']:
        raise SystemExit('Modified S1 v2 compatibility source; preserve: '+name)
print('READY')
'''
    result = subprocess.run([*ssh,host,shlex.join(['/usr/bin/python3','-c',check,
                             REMOTE_PACKAGE,COMPAT_VERSION])],capture_output=True,
                            text=True,timeout=20,check=True)
    if result.stdout.strip() == 'READY':
        print('REUSE S1 v2 USB0/1 compatibility',flush=True)
        return
    if running:
        raise RuntimeError('S1 v2 service is running; cannot patch live source')
    path = Path(archive or os.environ.get('GO2_S1_V2_ARCHIVE', root/DEFAULT_ARCHIVE))
    with zipfile.ZipFile(path) as bundle:
        payload = compatibility_payload(root,bundle)
    script = r'''
import base64,hashlib,json,pathlib,shutil,subprocess,sys
root=pathlib.Path(sys.argv[1]);d=json.loads(sys.stdin.read());files=d['files']
active=subprocess.run(['pgrep','-f','[r]un_arm_integration.py|[r]un_go2_left_grasp.py'],
                      stdout=subprocess.DEVNULL)
if active.returncode!=1:raise SystemExit('Existing S1 owner; preserve, no patch')
for name,meta in files.items():
    p=root/name
    if p.exists() and hashlib.sha256(p.read_bytes()).hexdigest() not in (
            meta['original_sha256'],meta['sha256']):
        raise SystemExit('Unexpected v2 source; preserve: '+name)
for name,meta in files.items():
    p=root/name;content=base64.b64decode(meta['content'])
    if p.exists() and p.read_bytes()==content:continue
    if p.exists():
        backup=root/'.v2-compat-upstream'/name
        backup.parent.mkdir(parents=True,exist_ok=True)
        if not backup.exists():shutil.copy2(p,backup)
    p.parent.mkdir(parents=True,exist_ok=True)
    temp=p.with_suffix(p.suffix+'.compat-tmp');temp.write_bytes(content);temp.replace(p)
meta={name:{k:v for k,v in values.items() if k!='content'} for name,values in files.items()}
(root/'S1_V2_COMPAT.json').write_text(json.dumps(dict(version=d['version'],files=meta),indent=2)+'\n')
print('S1_V2_USB01_COMPAT_READY')
'''
    subprocess.run([*ssh,host,shlex.join(['/usr/bin/python3','-c',script,REMOTE_PACKAGE])],
                   input=json.dumps(dict(version=COMPAT_VERSION,files=payload)),
                   text=True,timeout=30,check=True)


def ensure_v2_package(root, ssh, host, *, running=False, archive=None):
    """Return installed v2 root, installing once if not present and no S1 owner runs."""
    target = REMOTE_PACKAGE
    check = ("import json,pathlib; p=pathlib.Path(" + repr(target) +
             ")/'BUILD_INFO.json'; "
             "print(json.loads(p.read_text()).get('version') if p.is_file() else 'MISSING')")
    probe = subprocess.run([*ssh, host, shlex.join(['/usr/bin/python3','-c',check])],
                           capture_output=True, text=True, timeout=20, check=True)
    if probe.stdout.strip() == PACKAGE_VERSION:
        ensure_v2_compat(root,ssh,host,running=running,archive=archive)
        print('REUSE S1 v2 package: '+target, flush=True)
        return target
    if running:
        raise RuntimeError('已有S1串口所有者，不能热替换S1版本；先结束现有任务并停止旧服务')
    if probe.stdout.strip() != 'MISSING':
        raise RuntimeError('S1部署目录已存在不同版本，未覆盖: '+target)
    path = Path(archive or os.environ.get('GO2_S1_V2_ARCHIVE', root / DEFAULT_ARCHIVE))
    if not path.is_file():
        raise RuntimeError('缺少S1 v2压缩包，请放在 '+str(path)+' 或设置 GO2_S1_V2_ARCHIVE')
    with zipfile.ZipFile(path) as bundle:
        bad = bundle.testzip()
        if bad:
            raise RuntimeError('S1 v2 ZIP损坏: '+bad)
        info = json.loads(bundle.read('s1-arm-control/BUILD_INFO.json'))
        if info.get('version') != PACKAGE_VERSION:
            raise RuntimeError('S1压缩包不是v2_auto_integration')
        for entry in bundle.namelist():
            posix = Path(entry)
            if not entry.startswith('s1-arm-control/') or '..' in posix.parts:
                raise RuntimeError('S1压缩包路径不合法: '+entry)
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    name = '/tmp/go2-s1-v2-'+digest[:16]+'.zip'
    print('INSTALL S1 v2 once: '+str(path)+' -> '+target,flush=True)
    subprocess.run(['scp','-i',ssh[2],'-o','BatchMode=yes',str(path),host+':'+name],
                   check=True,timeout=180)
    script = """
import hashlib,json,os,pathlib,shutil,subprocess,sys,zipfile
if subprocess.run(['pgrep','-f','[r]un_arm_integration.py|[r]un_go2_left_grasp.py'],
                  stdout=subprocess.DEVNULL).returncode!=1:
    raise SystemExit('Existing S1 owner; preserve, no installation')
archive=pathlib.Path(sys.argv[1])
if hashlib.sha256(archive.read_bytes()).hexdigest()!=sys.argv[3]:
    raise SystemExit('Uploaded ZIP hash mismatch')
target=pathlib.Path(sys.argv[2])
temp=target.with_name(target.name+'.installing')
if target.exists() or temp.exists():
    raise SystemExit('S1目标目录已存在；未覆盖')
with zipfile.ZipFile(archive) as z:
    for item in z.infolist():
        p=pathlib.PurePosixPath(item.filename)
        if not item.filename.startswith('s1-arm-control/') or '..' in p.parts:
            raise SystemExit('S1 ZIP非法路径')
    temp.mkdir()
    for item in z.infolist():
        parts=pathlib.PurePosixPath(item.filename).parts[1:]
        if not parts: continue
        out=temp.joinpath(*parts)
        if item.is_dir():out.mkdir(parents=True,exist_ok=True);continue
        out.parent.mkdir(parents=True,exist_ok=True)
        with z.open(item) as f,out.open('wb') as dest:shutil.copyfileobj(f,dest)
if json.loads((temp/'BUILD_INFO.json').read_text())['version']!='v2_auto_integration':
    raise SystemExit('S1 v2安装版本验证失败')
for line in (temp/'SHA256SUMS').read_text().splitlines():
    digest,name=line.split('  ',1)
    if hashlib.sha256((temp/name).read_bytes()).hexdigest()!=digest:
        raise SystemExit('S1 v2 original manifest mismatch: '+name)
(temp/'S1_V2_DELIVERY.json').write_text(json.dumps(dict(archive_sha256=sys.argv[3])))
os.replace(temp,target)
archive.unlink(missing_ok=True)
print('S1_V2_PACKAGE_READY',target)
"""
    subprocess.run([*ssh,host,shlex.join(['/usr/bin/python3','-c',script,name,target,digest])],
                   check=True,timeout=120)
    ensure_v2_compat(root,ssh,host,running=False,archive=path)
    return target
