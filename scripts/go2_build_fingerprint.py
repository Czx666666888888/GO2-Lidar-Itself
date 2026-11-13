#!/usr/bin/env python3
"""Fingerprint only build inputs; documentation commits do not invalidate overlays."""
import hashlib
from pathlib import Path
import sys


def fingerprint(root):
    root = Path(root)
    digest = hashlib.sha256()
    for path in sorted(root.rglob('*')):
        if not path.is_file() or any(p in ('__pycache__', '.pytest_cache', 'test', 'tests')
                                     for p in path.relative_to(root).parts):
            continue
        if any(part.endswith('.egg-info') for part in path.parts):
            continue
        if path.suffix in ('.pyc', '.pyo'):
            continue
        digest.update(str(path.relative_to(root)).encode()+b'\0'+path.read_bytes())
    return digest.hexdigest()


if __name__ == '__main__':
    print(fingerprint(sys.argv[1]))
