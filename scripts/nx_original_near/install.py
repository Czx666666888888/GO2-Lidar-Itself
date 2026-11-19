#!/usr/bin/env python3
"""Install reviewed near-stage isolation with hash guards and rollback; no motion."""
import ast
import hashlib
import json
import os
from pathlib import Path
import tempfile


def install(stage, root):
    manifest = json.loads((stage/'manifest.json').read_text())
    writes = []
    module = 'camera_ws/src/go2_science_perception/go2_science_perception/mission_manager.py'
    installed = '.nx/install/go2_science_perception/lib/python3.8/site-packages/go2_science_perception/mission_manager.py'
    # Validate every destination before making any change.
    for rel, hashes in manifest.items():
        payload = (stage/rel).read_bytes()
        assert hashlib.sha256(payload).hexdigest() == hashes['after'], rel
        ast.parse(payload)
        for dest in ([rel, installed] if rel == module else [rel]):
            path = root/dest
            old = path.read_bytes()
            assert hashlib.sha256(old).hexdigest() == hashes['before'], 'baseline changed: '+dest
            writes.append((dest, old, payload, path.stat().st_mode & 0o777))
    active = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            argv = (proc/'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if any(Path(arg.decode(errors='replace')).name in
               ('run_map_anchor_experiment.py', 'go2_full_test.py', 'science_target_approach')
               for arg in argv if arg):
            active.append(proc.name)
    if active:
        raise RuntimeError('FULL/controller still running; files not changed: '+str(active))
    backup = Path(tempfile.mkdtemp(prefix='original_near_backup.', dir=root.parent))
    for rel, old, _, _ in writes:
        saved = backup/rel
        saved.parent.mkdir(parents=True, exist_ok=True)
        saved.write_bytes(old)
    try:
        for rel, old, payload, mode in writes:
            path = root/rel
            fd, name = tempfile.mkstemp(prefix='.original-near.', dir=path.parent)
            with os.fdopen(fd, 'wb') as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            os.chmod(name, mode)
            os.replace(name, path)
            assert path.read_bytes() == payload
    except BaseException:
        for rel, old, _, _ in writes:
            (root/rel).write_bytes(old)
        raise
    (backup/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print('BACKUP', backup)
    for rel, _, payload, _ in writes:
        print('READBACK', rel, hashlib.sha256(payload).hexdigest())


if __name__ == '__main__':
    install(Path(__file__).resolve().parent,
            Path('/home/unitree/go2_full_nx/releases/599c86c/source'))
