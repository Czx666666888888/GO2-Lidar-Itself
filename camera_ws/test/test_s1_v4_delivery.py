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
        configuration_changes = {name for name in payload['files'] if name.startswith('config/')}
        assert configuration_changes == {delivery.LEFT_GRASP_CONFIG}
        assert not any(name.startswith(('calibration/', 'vendor/')) for name in payload['files'])
        original = bundle.read('s1-arm-control/' + delivery.LEFT_GRASP_CONFIG)
        updated = base64.b64decode(payload['files'][delivery.LEFT_GRASP_CONFIG]['content'])
        expected = original.replace(b'"maximum_track_error_rad": 0.08,',
                                    b'"maximum_track_error_rad": 0.12,', 1)
        assert updated == expected
        assert updated != original
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
def test_accepting_new_task_clears_old_result_but_rejecting_busy_task_does_not(session, task):
    previous = dict(last_result='old failure', last_log='/old/run.json',
                    last_exit_code=1, error='old failure')
    session._update(**previous)
    request = session.request_grasp if task == 'grasp' else lambda: session.request_task('dump', {})
    assert request()['accepted']
    state = session.snapshot()
    assert state['state'] == 'QUEUED'
    assert all(state[key] is None for key in previous)

    current = dict(last_result='current failure', last_log='/current/run.json',
                   last_exit_code=2, error='current failure')
    session._update(**current)
    assert not request()['accepted']
    assert all(session.snapshot()[key] == value for key, value in current.items())


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


@pytest.mark.parametrize('active_owner', [False, True])
@pytest.mark.parametrize('previous_version', delivery.PRIOR_VERSIONS)
def test_upgrade_preserves_every_other_resource(installed, payload, active_owner, previous_version):
    target, expected = installed
    previous = delivery.delivery_payload(ROOT, version=previous_version)[1]
    with zipfile.ZipFile(ROOT / delivery.ARCHIVE) as bundle:
        config = delivery.LEFT_GRASP_CONFIG
        (target / config).write_bytes(bundle.read('s1-arm-control/' + config))
    for name, item in previous['files'].items():
        (target / name).write_bytes(base64.b64decode(item.pop('content')))
    (target / 'S1_V2_COMPAT.json').write_text(json.dumps(previous))
    check = check_installed(target, dict(expected, __previous__={previous['version']: previous}))
    assert check.returncode == 0, check.stderr
    assert check.stdout.strip() == 'UPGRADE ' + previous['version']
    before = {name: (target / name).read_bytes() for name in expected}
    if active_owner:
        import os
        pgrep = Path(os.environ['PATH'].split(os.pathsep)[0]) / 'pgrep'
        pgrep.write_text('#!/bin/sh\nexit 0\n')
    result = subprocess.run([sys.executable, '-c', delivery.UPGRADE, str(target)],
                            input=json.dumps(dict(previous=previous, current=copy.deepcopy(payload))),
                            text=True, capture_output=True)
    changed = {name for name in expected if (target / name).read_bytes() != before[name]}
    if active_owner:
        assert result.returncode != 0 and 'Existing S1 owner' in result.stderr
        assert not changed
        assert json.loads((target / 'S1_V2_COMPAT.json').read_text()) == previous
    else:
        assert result.returncode == 0, result.stderr
        assert delivery.LEFT_GRASP_CONFIG in changed
        changed.remove(delivery.LEFT_GRASP_CONFIG)
        if previous_version != delivery.PRIOR_VERSIONS[-1]:
            assert 's1_control/integration.py' in changed
            changed.remove('s1_control/integration.py')
        if previous_version == delivery.PRIOR_VERSIONS[0]:
            name = 'scripts/run_go2_visual_alignment.py'
            assert changed == {name}
            assert (target / name).read_bytes() == before[name].replace(
                b'setdefault("secure_force_nm", 0.40)', b'setdefault("secure_force_nm", 0.80)', 1)
        else:
            assert not changed
        assert check_installed(target, expected).returncode == 0
