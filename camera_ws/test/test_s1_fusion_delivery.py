"""Install and verify the current fusion package offline."""
import base64
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
import s1_fusion_delivery as fusion
from s1_v4_delivery import CHECK, INSTALL


def test_fusion_installs_with_original_grasp_and_plan_resources(tmp_path, monkeypatch):
    archive, payload = fusion.delivery_payload(ROOT)
    with ZipFile(archive) as bundle:
        original = bundle.read('s1-arm-control/scripts/run_go2_visual_alignment.py')
        arm = bundle.read('s1-arm-control/s1_control/arm.py')
    assert b'j6_max_track_err_rad=0.12' in original
    assert b'setdefault("secure_force_nm", 0.50)' in original
    assert b'last_result=None, last_log=None, last_exit_code=None, error=None' in base64.b64decode(
        payload['files']['s1_control/integration.py']['content'])
    assert not any(name.startswith(('config/', 'calibration/', 'data/', 'vendor/'))
                   for name in payload['files'])

    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    pgrep = bin_dir / 'pgrep'
    pgrep.write_text('#!/bin/sh\nexit 1\n')
    pgrep.chmod(0o755)
    monkeypatch.setenv('PATH', str(bin_dir) + os.pathsep + os.environ['PATH'])
    incoming = tmp_path / archive.name
    shutil.copyfile(archive, incoming)
    target = tmp_path / 'installed'
    subprocess.run([sys.executable, '-c', INSTALL, str(incoming), str(target)],
                   input=json.dumps(payload), text=True, check=True)
    assert (target / 'scripts/run_go2_visual_alignment.py').read_bytes() == original
    assert (target / 's1_control/arm.py').read_bytes() == arm
    expected = {name: {key: value for key, value in item.items() if key != 'content'}
                for name, item in payload['files'].items()}
    result = subprocess.run([sys.executable, '-c', CHECK, str(target),
                             fusion.COMPAT_VERSION, fusion.ARCHIVE_SHA256],
                            input=json.dumps(expected), text=True, capture_output=True, check=True)
    assert result.stdout.strip() == 'READY'
