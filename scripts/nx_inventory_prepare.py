#!/usr/bin/env python3
"""Prepare inventory diagnostics only; retain camera readiness requirements."""
import argparse
import ast
import difflib
import hashlib
import json
from pathlib import Path


def prepare(source, output):
    rel = Path('scripts/go2_full_test.py')
    before = (source/rel).read_text()
    old = "            inventory=json.loads(self.host_run(shlex.join([ARM_PY,'-c',INVENTORY_SCRIPT]),\n                                        capture_output=True,text=True).stdout)"
    assert before.count(old) == 1
    after = before.replace('import argparse\n', 'import argparse\nfrom nx_inventory import read_inventory\n', 1)
    after = after.replace(old, '            inventory=read_inventory(self.host_run, ARM_PY, INVENTORY_SCRIPT, self.folder)', 1)
    entries = {str(rel): (before, after), 'scripts/nx_inventory.py':
               (None, Path(__file__).with_name('nx_inventory.py').read_text())}
    manifest, patch = {}, []
    output.mkdir(parents=True, exist_ok=True)
    for name, (original, payload) in entries.items():
        ast.parse(payload)
        target = output/name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(payload)
        manifest[name] = dict(before=hashlib.sha256(original.encode()).hexdigest() if original else None,
                              after=hashlib.sha256(payload.encode()).hexdigest())
        patch.extend(difflib.unified_diff((original or '').splitlines(True), payload.splitlines(True),
                     fromfile='a/'+name if original else '/dev/null', tofile='b/'+name, n=0))
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (output/'change.patch').write_text(''.join(patch))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    prepare(args.source, args.output)
