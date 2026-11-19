#!/usr/bin/env python3
"""Hash-guarded source update with backups; never launch the robot."""
import ast
import hashlib
import json
import os
from pathlib import Path
import tempfile


def install(stage, root):
    manifest = json.loads((stage/'manifest.json').read_text())
    writes = []
    for rel, hashes in manifest.items():
        dest = root/rel
        old = dest.read_bytes() if dest.exists() else None
        assert (hashlib.sha256(old).hexdigest() if old is not None else None) == hashes['before'], rel
        payload = (stage/rel).read_bytes()
        assert hashlib.sha256(payload).hexdigest() == hashes['after'], rel
        ast.parse(payload)
        writes.append((rel, old, payload, dest.stat().st_mode & 0o777 if old is not None else 0o644))
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            argv = (proc/'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if any(Path(arg.decode(errors='replace')).name == 'go2_full_test.py' for arg in argv if arg):
            raise RuntimeError('FULL still running: '+proc.name)
    backup = Path(tempfile.mkdtemp(prefix='timed_speed_backup.', dir=root.parent))
    for rel, old, _, _ in writes:
        if old is not None:
            target = backup/rel
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(old)
    try:
        for rel, old, payload, mode in writes:
            dest = root/rel
            fd, name = tempfile.mkstemp(prefix='.timed-speed.', dir=dest.parent)
            with os.fdopen(fd, 'wb') as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            os.chmod(name, mode)
            os.replace(name, dest)
            assert dest.read_bytes() == payload
    except BaseException:
        for rel, old, _, _ in writes:
            if old is None:
                (root/rel).unlink(missing_ok=True)
            else:
                (root/rel).write_bytes(old)
        raise
    (backup/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print('BACKUP', backup)
    for rel, _, payload, _ in writes:
        print('READBACK', rel, hashlib.sha256(payload).hexdigest())


if __name__ == '__main__':
    install(Path(__file__).resolve().parent,
            Path('/home/unitree/go2_full_nx/releases/599c86c/source'))
