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

PACKAGE_VERSION = 'v2_auto_integration'
DEFAULT_ARCHIVE = 'S1_arm_integration_20261114_auto_v2.zip'
REMOTE_PACKAGE = '/home/unitree/s1-arm-control-v2'


def ensure_v2_package(root, ssh, host, *, running=False, archive=None):
    """Return installed v2 root, installing once if not present and no S1 owner runs."""
    target = REMOTE_PACKAGE
    check = ("import json,pathlib; p=pathlib.Path(" + repr(target) +
             ")/'BUILD_INFO.json'; "
             "print(json.loads(p.read_text()).get('version') if p.is_file() else 'MISSING')")
    probe = subprocess.run([*ssh, host, shlex.join(['/usr/bin/python3','-c',check])],
                           capture_output=True, text=True, timeout=20, check=True)
    if probe.stdout.strip() == PACKAGE_VERSION:
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
import json,os,pathlib,shutil,sys,zipfile
archive=pathlib.Path(sys.argv[1])
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
os.replace(temp,target)
archive.unlink(missing_ok=True)
print('S1_V2_PACKAGE_READY',target)
"""
    subprocess.run([*ssh,host,shlex.join(['/usr/bin/python3','-c',script,name,target])],
                   check=True,timeout=120)
    return target
