"""Offline safety/lifecycle checks: no ROS, SSH or robot required."""
import contextlib
import importlib.util
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location(
    'go2_full_test', Path(__file__).resolve().parents[1]/'scripts/go2_full_test.py')
FLOW = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(FLOW)


class LifecycleTests(unittest.TestCase):
    def test_default_and_dock_plan_never_spawn_or_connect(self):
        for mode in ('full', 'dock'):
            with patch.object(sys, 'argv', ['go2_full_test.py', '--mode', mode]), \
                    patch.object(FLOW.subprocess, 'run', side_effect=AssertionError('run')), \
                    patch.object(FLOW.subprocess, 'Popen', side_effect=AssertionError('spawn')), \
                    contextlib.redirect_stdout(io.StringIO()) as output:
                FLOW.main()
            self.assertIn('NOT VERIFIED', output.getvalue())

    def test_busy_or_fault_arm_never_allows_navigation(self):
        for state in ({}, {'state': 'FAULT_HOLD'},
                      {'state': 'ZERO_HOLD', 'busy': True, 'navigation_allowed': True},
                      {'state': 'ZERO_HOLD', 'busy': False, 'navigation_allowed': False}):
            self.assertFalse(FLOW.zero_hold(state))
        self.assertTrue(FLOW.zero_hold(dict(state='ZERO_HOLD', busy=False,
                                            navigation_allowed=True)))

    def test_remote_camera_early_exit_is_failure(self):
        with tempfile.TemporaryDirectory() as folder:
            script = Path(folder)/'camera.sh'
            script.write_text('exit 7\n')
            p = subprocess.Popen([sys.executable, '-c', FLOW.CAMERA_OWNER, str(script)],
                                 stdin=subprocess.PIPE)
            try:
                self.assertEqual(p.wait(timeout=4), 7)
            finally:
                p.stdin.close()
                if p.poll() is None:
                    p.kill()
                    p.wait()

    def test_connection_eof_stops_owned_camera_and_preserves_other_process(self):
        with tempfile.TemporaryDirectory() as folder:
            pidfile = Path(folder)/'camera.pid'
            script = Path(folder)/'camera.sh'
            script.write_text('echo $$ > '+str(pidfile)+'\nexec sleep 60\n')
            other = subprocess.Popen(['sleep', '60'])
            p = subprocess.Popen([sys.executable, '-c', FLOW.CAMERA_OWNER, str(script)],
                                 stdin=subprocess.PIPE)
            try:
                deadline = time.monotonic()+4
                while not pidfile.exists() and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertTrue(pidfile.exists())
                camera_pid = int(pidfile.read_text())
                p.stdin.close()
                self.assertEqual(p.wait(timeout=12), 0)
                with self.assertRaises(ProcessLookupError):
                    os.kill(camera_pid, 0)
                self.assertIsNone(other.poll())
            finally:
                if not p.stdin.closed:
                    p.stdin.close()
                if p.poll() is None:
                    p.kill()
                    p.wait()
                other.terminate()
                other.wait()


if __name__ == '__main__':
    unittest.main()
