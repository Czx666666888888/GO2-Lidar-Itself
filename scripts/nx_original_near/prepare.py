#!/usr/bin/env python3
"""Prepare a scoped NX patch; never start robot processes."""
import argparse
import ast
import difflib
import hashlib
import json
from pathlib import Path


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('source mismatch: '+old[:100])
    return text.replace(old, new, 1)


def transform_manager(text):
    text = replace_once(text, '    def command_clear(self, command):\n',
        '    def command_clear(self, command):\n'
        '        if getattr(getattr(self, "run", None), "navigation_suspended", False):\n'
        '            return True  # Original controller and Sport gate own near motion.\n')
    text = replace_once(text, "        conflicts = [topic for topic in NAV_TOPICS",
        '        if getattr(getattr(self, "run", None), "navigation_suspended", False):\n'
        '            self.original_near_step()\n'
        '            return\n'
        '        conflicts = [topic for topic in NAV_TOPICS')
    text = replace_once(text, '    def begin_stop_confirm(self, now):\n'
        "        self.change('STOP_CONFIRM')\n        self.hold_navigation()\n",
        '    def begin_stop_confirm(self, now):\n'
        '        self.hold_navigation()\n'
        '        self.run.suspend_navigation()\n'
        "        self.change('STOP_CONFIRM')\n")
    text = replace_once(text, '        self.deadline = now + 12.\n',
        '        self.deadline = time.monotonic() + 12.\n')
    marker = '    def drive(self, distance):\n'
    method = '''    def original_near_step(self):
        """Observe the existing approach controller without running navigation gates."""
        report = self.run.control
        if time.monotonic() - self.run.last_control_received > 1.:
            self.fail('Original approach status reception stopped')
            return
        state = report.get('state')
        complete = report.get('grasp_docking_complete', False)
        if state in NEAR_STATES or complete:
            side = report.get('grasp_side')
            expected_side = (self.active_side or self.side_hint or
                             (self.stage.side if self.stage is not None else None))
            if report.get('target_id') != self.anchor['id']:
                self.fail('Original approach target identity mismatch')
                return
            if side not in self.sides or (expected_side is not None and side != expected_side):
                self.fail('Original approach grasp side mismatch')
                return
        if state in ('STOP', 'FAULT'):
            self.change('GRASP' if complete else 'IDLE', 'GRASP' if complete else 'IDLE')
            self.finished = True
            return
        if self.state == 'STOP_CONFIRM':
            if state in NEAR_STATES:
                self.active_side = report.get('grasp_side')
                self.run.event('ORIGINAL_APPROACH_SELECTED', target_id=report.get('target_id'),
                               side=self.active_side)
                self.change('APPROACH', 'APPROACH')
            elif time.monotonic() > self.deadline:
                self.fail('Original STOP_CONFIRM did not admit approach')
            return
        if self.state in ('APPROACH', 'DOCK'):
            if state not in NEAR_STATES:
                self.fail('Original approach left the near-control state')
                return
            if report.get('docking_phase') and self.state != 'DOCK':
                self.change('DOCK', 'DOCK')
            self.arbiter.enabled = state in NEAR_STATES

'''
    return replace_once(text, marker, method+marker)


def transform_runner(text):
    text = replace_once(text, '        self.navigation_released = False\n',
        '        self.navigation_released = False\n        self.navigation_suspended = False\n')
    text = replace_once(text, '                    or self.nav_process.poll() is not None\n',
        '                    or (not self.navigation_suspended and self.nav_process.poll() is not None)\n')
    marker = '    def release_navigation(self):\n'
    method = '''    def suspend_navigation(self):
        """Stop only this run's planner launch; retain Point-LIO, camera and control."""
        if self.navigation_suspended:
            return
        self.mission.arbiter.flush()
        self.event('NEAR_NAVIGATION_STOP_START')
        # Keep callbacks and zero-speed heartbeat alive while planners exit.
        for stop_signal, timeout in ((signal.SIGINT, 3.), (signal.SIGTERM, 2.),
                                     (signal.SIGKILL, 1.)):
            try:
                os.killpg(self.nav_process.pid, stop_signal)
            except ProcessLookupError:
                pass
            deadline = time.monotonic() + timeout
            while self.nav_process.poll() is None and time.monotonic() < deadline:
                self.spin(.02)
            if self.nav_process.poll() is not None:
                break
        if self.nav_process.poll() is None:
            raise RuntimeError('Owned navigation launch did not exit')
        topics = ('/mission/nbv_goal', '/mission/far_way_point', '/nav/path', '/nav/cmd_vel_raw')
        deadline = time.monotonic() + 3.
        while True:
            remaining = [topic for topic in topics if self.node.count_publishers(topic)]
            if not remaining:
                break
            if time.monotonic() > deadline:
                raise RuntimeError('Navigation publishers did not exit: '+str(remaining))
            self.spin(.02)
        self.navigation_suspended = True
        self.event('NEAR_NAVIGATION_STOPPED', original_controller=True)

'''
    return replace_once(text, marker, method+marker)


def prepare(source, output):
    changes = {}
    for rel, transform in (
        ('camera_ws/scripts/run_map_anchor_experiment.py', transform_runner),
        ('camera_ws/src/go2_science_perception/go2_science_perception/mission_manager.py',
         transform_manager),
    ):
        before = (source/rel).read_text()
        after = transform(before)
        ast.parse(after)
        changes[rel] = (before, after)
    output.mkdir(parents=True, exist_ok=True)
    manifest = {}
    patches = []
    for rel, (before, after) in changes.items():
        target = output/rel
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(after)
        manifest[rel] = {'before': hashlib.sha256(before.encode()).hexdigest(),
                         'after': hashlib.sha256(after.encode()).hexdigest()}
        patches.extend(difflib.unified_diff(before.splitlines(True), after.splitlines(True),
                                           fromfile='a/'+rel, tofile='b/'+rel, n=0))
    (output/'change.patch').write_text(''.join(patches))
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    prepare(args.source, args.output)
