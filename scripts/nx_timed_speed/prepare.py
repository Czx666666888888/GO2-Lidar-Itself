#!/usr/bin/env python3
"""Prepare only the NX timed speed source changes, without executing robot code."""
import argparse
import ast
import difflib
import hashlib
import json
from pathlib import Path


def once(text, old, new):
    assert text.count(old) == 1, old[:100]
    return text.replace(old, new, 1)


def prepare(source, output):
    scripts = Path('scripts')
    timed = scripts/'nx_timed_move_only/go2_reactive_avoid'
    files = {rel: (source/rel).read_text() for rel in
             (scripts/'go2_full_test.py', timed/'node.py', timed/'timed_straight.py')}
    changed = dict(files)
    rel = scripts/'go2_full_test.py'
    s = changed[rel]
    s = once(s, 'import argparse\n',
             'import argparse\nfrom nx_timed_options import normalize_timed_args, validate_timed_speed\n')
    start = s.index("    parser.add_argument('--timed-straight'")
    end = s.index('    if args.timed_straight is not None:', start)
    s = s[:start]+'''    parser.add_argument('--timed-straight', type=float, metavar='SECONDS',
                        help='NX FULL 前置时长：--3 为 3 秒，小数用 --timed-straight 2.5')
    parser.add_argument('--timed-speed', type=float, metavar='MPS',
                        help='NX 前置速度 0.10–1.00 m/s，可简写 --1.0；默认 1.0')
    args = parser.parse_args(normalize_timed_args(sys.argv[1:], parser))
    validate_timed_speed(args, parser)
'''+s[end:]
    s = once(s, "'--timed', str(self.args.timed_straight), '--timed-ready', str(ready),",
             "'--timed', str(self.args.timed_straight), '--timed-ready', str(ready),\n"
             "                   '--timed-speed', str(self.args.timed_speed),")
    s = once(s, "'前置段目标速度 1.00 m/s，按 IMU 航向计时直行。')",
             "f'前置段目标速度 {args.timed_speed:.2f} m/s，按 IMU 航向计时直行。')")
    s = once(s, "print(f'预检 {timed:g} 秒计时直行。",
             "print(f'预检 {timed:g} 秒、{self.args.timed_speed:.2f} m/s 计时直行。")
    s = once(s, "        child.stdin.flush()\n        deadline = time.monotonic() + self.args.timed_straight",
             "        child.stdin.flush()\n"
             "        print('NX_START_ACCEPTED：启动确认已发送，可拔 PC 侧网线；NX 与 GO2 的连线须保留。', flush=True)\n"
             "        deadline = time.monotonic() + self.args.timed_straight")
    changed[rel] = s
    rel = timed/'timed_straight.py'
    s = once(changed[rel], 'def __init__(self, seconds, ready):',
             'def __init__(self, seconds, ready, speed=SPEED_CAP):')
    s = once(s, '        self.seconds = seconds\n',
             '        if not math.isfinite(speed) or not START_SPEED <= speed <= SPEED_CAP:\n'
             '            raise ValueError("直行速度须在 0.10–1.00 m/s 内")\n'
             '        self.speed = speed\n        self.seconds = seconds\n')
    changed[rel] = once(s, '        speed = SPEED_CAP\n', '        speed = self.speed\n')
    rel = timed/'node.py'
    s = once(changed[rel], '    parser.add_argument("--timed-ready",',
             '    parser.add_argument("--timed-speed", type=float, default=SPEED_CAP)\n'
             '    parser.add_argument("--timed-ready",')
    s = once(s, 'timed = TimedStraight(args.timed, args.timed_ready)',
             'timed = TimedStraight(args.timed, args.timed_ready, args.timed_speed)')
    changed[rel] = once(s, '    max_speed = SPEED_CAP if timed else',
                        '    max_speed = timed.speed if timed else')
    helper = scripts/'nx_timed_options.py'
    files[helper] = None
    changed[helper] = (Path(__file__).resolve().parents[1]/helper.name).read_text()
    output.mkdir(parents=True, exist_ok=True)
    manifest, patch = {}, []
    for rel, after in changed.items():
        ast.parse(after)
        before = files[rel]
        dest = output/rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_text(after)
        manifest[str(rel)] = dict(before=hashlib.sha256(before.encode()).hexdigest() if before else None,
                                 after=hashlib.sha256(after.encode()).hexdigest())
        patch.extend(difflib.unified_diff((before or '').splitlines(True), after.splitlines(True),
                     fromfile='a/'+str(rel) if before else '/dev/null', tofile='b/'+str(rel), n=0))
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (output/'change.patch').write_text(''.join(patch))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    prepare(args.source, args.output)
