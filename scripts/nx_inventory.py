"""Preserve device inventory errors instead of printing the embedded shell command."""
import json
from pathlib import Path
import shlex
import subprocess


def read_inventory(host_run, python, script, folder):
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    error_log = folder/'inventory_error.log'

    def failure(reason, stdout='', stderr=''):
        def decode(value):
            return value.decode(errors='replace') if isinstance(value, bytes) else (value or '')
        stdout, stderr = decode(stdout), decode(stderr)
        error_log.write_text(reason+'\nSTDOUT:\n'+stdout+'\nSTDERR:\n'+stderr+'\n')
        detail = stderr.strip().splitlines()[-1] if stderr.strip() else reason
        raise RuntimeError('设备清点失败：'+detail+'；原始错误：'+str(error_log)) from None

    try:
        result = host_run(shlex.join([python, '-c', script]), capture_output=True, text=True)
    except subprocess.CalledProcessError as exc:
        failure('inventory exit '+str(exc.returncode), exc.stdout, exc.stderr)
    except subprocess.TimeoutExpired as exc:
        failure('inventory timeout '+str(exc.timeout)+'s', exc.stdout, exc.stderr)
    try:
        data = json.loads(result.stdout)
        if not isinstance(data, dict):
            raise ValueError('inventory JSON must be an object')
    except ValueError as exc:
        failure('invalid inventory output: '+str(exc), result.stdout, result.stderr)
    (folder/'inventory.json').write_text(json.dumps(data, indent=2)+'\n')
    return data
