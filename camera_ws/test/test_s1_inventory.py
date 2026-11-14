"""Regression checks for actual script identity; no device or motion access."""
import ast
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'scripts'))
from go2_full_test import INVENTORY_SCRIPT


def resolve_entry(argv, cwd='/home/unitree'):
    node = next(n for n in ast.parse(INVENTORY_SCRIPT).body
                if isinstance(n, ast.FunctionDef) and n.name == 'arm_entry')
    scope = dict(Path=Path)
    exec(compile(ast.Module(body=[node], type_ignores=[]), '<inventory>', 'exec'), scope)
    return scope['arm_entry'](argv, cwd)


@pytest.mark.parametrize('argv', [
    ['bash', '-c', INVENTORY_SCRIPT],
    ['python', '-c', INVENTORY_SCRIPT],
    ['python3', '-uB', '-c', 'run_arm_integration.py'],
    ['python3', '-m', 'run_arm_integration.py'],
    ['python3', 'other.py', 'run_go2_left_grasp.py'],
])
def test_embedded_script_name_is_not_service(argv):
    assert resolve_entry(argv) is None


@pytest.mark.parametrize('side', ['run_arm_integration.py', 'run_go2_left_grasp.py'])
def test_absolute_v2_entry_does_not_require_v2_cwd(side):
    entry = '/home/unitree/s1-arm-control-v2/scripts/'+side
    assert resolve_entry(['/home/unitree/s1-arm-py310/bin/python', '-uB', entry]) == Path(entry)


def test_relative_v2_and_actual_legacy_are_distinguished():
    argv = ['python3', '-u', 'scripts/run_arm_integration.py', '--dev', 'auto']
    assert resolve_entry(argv, '/home/unitree/s1-arm-control-v2').parent.parent.name == 's1-arm-control-v2'
    assert resolve_entry(argv, '/home/unitree/s1-arm-control').parent.parent.name == 's1-arm-control'
