"""Final-fusion integration against the pinned real ZIP; no hardware access."""
import base64
import copy
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from types import ModuleType, SimpleNamespace
import zipfile

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
import s1_v4_delivery as delivery


@pytest.fixture
def payload():
    return delivery.delivery_payload(ROOT)[1]


@pytest.fixture
def session(payload, monkeypatch):
    package = ModuleType('s1_v4_fixture')
    package.__path__ = []
    monkeypatch.setitem(sys.modules, package.__name__, package)
    for name in ('startup_status', 'integration'):
        module = ModuleType(package.__name__ + '.' + name)
        module.__package__ = package.__name__
        monkeypatch.setitem(sys.modules, module.__name__, module)
        exec(base64.b64decode(payload['files']['s1_control/' + name + '.py']['content']),
             module.__dict__)
    arm = SimpleNamespace(limits=SimpleNamespace(max_temp_c=55, max_track_err_rad=.08),
                          get_pos=lambda: [0.] * 7, get_vel=lambda: [0.] * 7,
                          get_temp=lambda: [25.] * 7)
    session = module.ArmIntegrationSession(arm)
    session.ready()
    return session


def test_archive_is_pinned_and_motion_resources_are_unchanged(payload):
    with zipfile.ZipFile(ROOT / delivery.ARCHIVE) as bundle:
        for name, item in payload['files'].items():
            assert hashlib.sha256(base64.b64decode(item['content'])).hexdigest() == item['sha256']
        # Upstream algorithms/configuration/SDK remain protected by the ZIP manifest.
        assert 's1_control/box_dump.py' not in payload['files']
        assert not any(name.startswith(('config/', 'calibration/', 'vendor/'))
                       for name in payload['files'])
        source = base64.b64decode(payload['files']['scripts/run_arm_integration.py']['content']).decode()
        assert 'task_commands_supported=["status", "grasp", "dump"]' in source
        assert 'elif request.get("command") == "dump"' in source
        assert 'task="dump"' in source
        assert bundle.read('s1-arm-control/config/go2_hinged_box_dump.json')


def test_grasp_latches_side_and_never_queues_dump(session):
    assert session.request_grasp('left', allow_missed=True)['accepted']
    assert session.allow_missed is True
    assert session.snapshot()['grasp_side'] == 'left'
    assert session.jobs.get_nowait() == 'grasp'
    assert not session.request_grasp('right')['accepted']
    assert not session.request_task('dump', {})['accepted']
    assert session.jobs.empty()


def test_dump_is_independent_and_never_replaces_busy_grasp(session):
    plan = {'plan': 'fixture'}
    assert session.request_task('dump', plan)['accepted']
    assert session.jobs.get_nowait() == ('dump', plan)
    assert session.snapshot()['navigation_allowed'] is False
    assert not session.request_grasp('left')['accepted']
    assert not session.request_task('dump', plan)['accepted']


@pytest.mark.parametrize('task', ['grasp', 'dump'])
def test_feedback_failure_rejects_both_tasks(session, task):
    session.arm.get_pos = lambda: [float('nan')] * 7
    reply = session.request_grasp('right') if task == 'grasp' else session.request_task('dump', {})
    assert not reply['accepted'] and session.jobs.empty()
    assert session.snapshot()['navigation_allowed'] is False


@pytest.mark.parametrize('running', [True, False])
def test_verified_existing_delivery_reuses_without_install(payload, monkeypatch, running):
    calls = []

    def run(*args, **kwargs):
        calls.append(args)
        return SimpleNamespace(stdout='READY\n')
    monkeypatch.setattr(delivery.subprocess, 'run', run)
    assert delivery.ensure_v4_package(ROOT, ['ssh'], 'fixture', running=running) == delivery.REMOTE_PACKAGE
    assert len(calls) == 1


def test_running_owner_is_preserved(monkeypatch):
    monkeypatch.setattr(delivery.subprocess, 'run',
                        lambda *a, **kw: SimpleNamespace(stdout='MISSING\n'))
    with pytest.raises(RuntimeError, match='running'):
        delivery.ensure_v4_package(ROOT, ['ssh'], 'fixture', running=True)


@pytest.fixture
def installed(payload, tmp_path, monkeypatch):
    # Execute the real installation/check scripts locally. pgrep is isolated.
    binary = tmp_path / 'bin'
    binary.mkdir()
    pgrep = binary / 'pgrep'
    pgrep.write_text('#!/bin/sh\nexit 1\n')
    pgrep.chmod(0o755)
    import os
    monkeypatch.setenv('PATH', str(binary) + os.pathsep + os.environ['PATH'])
    archive = tmp_path / 'delivery.zip'
    shutil.copyfile(ROOT / delivery.ARCHIVE, archive)
    target = tmp_path / 'installed'
    subprocess.run([sys.executable, '-c', delivery.INSTALL, str(archive), str(target)],
                   input=json.dumps(copy.deepcopy(payload)), text=True, check=True)
    expected = {name: {k: v for k, v in item.items() if k != 'content'}
                for name, item in payload['files'].items()}
    return target, expected


def check_installed(target, expected):
    return subprocess.run([sys.executable, '-c', delivery.CHECK, str(target),
                           delivery.COMPAT_VERSION, delivery.ARCHIVE_SHA256],
                          input=json.dumps(expected), text=True, capture_output=True)


def test_atomic_install_and_full_manifest_recheck(installed):
    target, expected = installed
    result = check_installed(target, expected)
    assert result.returncode == 0 and result.stdout.strip() == 'READY', result.stderr
    assert (target / '.v2-compat-upstream/scripts/run_arm_integration.py').exists()


@pytest.mark.parametrize('name', ['scripts/run_arm_integration.py',
                                  'config/go2_hinged_box_dump.json'])
def test_changed_adapter_or_bound_plan_is_preserved(installed, name):
    target, expected = installed
    (target / name).write_text('local modification')
    result = check_installed(target, expected)
    assert result.returncode != 0 and 'preserve' in result.stderr
    assert (target / name).read_text() == 'local modification'


def test_installer_race_with_new_owner_never_writes(payload, tmp_path, monkeypatch):
    binary = tmp_path / 'bin'
    binary.mkdir()
    pgrep = binary / 'pgrep'
    pgrep.write_text('#!/bin/sh\nexit 0\n')
    pgrep.chmod(0o755)
    import os
    monkeypatch.setenv('PATH', str(binary) + os.pathsep + os.environ['PATH'])
    target = tmp_path / 'must-not-exist'
    result = subprocess.run([sys.executable, '-c', delivery.INSTALL,
                             str(ROOT / delivery.ARCHIVE), str(target)],
                            input=json.dumps(payload), text=True, capture_output=True)
    assert result.returncode != 0 and 'Existing S1 owner' in result.stderr
    assert not target.exists() and not target.with_name(target.name + '.installing').exists()
