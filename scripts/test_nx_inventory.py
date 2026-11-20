"""No device access: test captured inventory subprocess outcomes."""
import json
import subprocess
from types import SimpleNamespace

import pytest

from nx_inventory import read_inventory


def test_success_preserves_actual_inventory(tmp_path):
    data = dict(cameras=['camera-1'], arm=[123])
    def host(command, **kwargs):
        assert kwargs == dict(capture_output=True, text=True)
        return SimpleNamespace(stdout=json.dumps(data), stderr='')
    assert read_inventory(host, '/python', 'script', tmp_path) == data
    assert json.loads((tmp_path/'inventory.json').read_text()) == data


@pytest.mark.parametrize('error,expected', [
    (subprocess.CalledProcessError(1, 'hidden-long-command', stderr=
        'Traceback\nRuntimeError: UVCIOC_CTRL_QUERY Protocol error'), 'Protocol error'),
    (subprocess.CalledProcessError(2, 'hidden-long-command'), 'inventory exit 2'),
    (subprocess.TimeoutExpired('hidden-long-command', 20, stderr=b'USB busy'), 'USB busy')])
def test_error_is_short_and_original_stderr_is_saved(tmp_path, error, expected):
    def host(*args, **kwargs):
        raise error
    with pytest.raises(RuntimeError, match=expected) as caught:
        read_inventory(host, '/python', 'script', tmp_path)
    assert 'hidden-long-command' not in str(caught.value)
    assert expected in (tmp_path/'inventory_error.log').read_text()
    assert not (tmp_path/'inventory.json').exists()


@pytest.mark.parametrize('output', ['not-json', '[]'])
def test_invalid_output_does_not_become_ready(tmp_path, output):
    with pytest.raises(RuntimeError, match='invalid inventory output'):
        read_inventory(lambda *a, **kw: SimpleNamespace(stdout=output, stderr=''),
                       '/python', 'script', tmp_path)
    assert output in (tmp_path/'inventory_error.log').read_text()
