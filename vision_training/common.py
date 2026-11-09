"""Path, metadata and reproducibility helpers; no torch import on validation."""

import hashlib
import importlib.metadata
import json
import platform
from pathlib import Path
import subprocess

import yaml


def read_yaml(path):
    value = yaml.safe_load(Path(path).read_text())
    if not isinstance(value, dict):
        raise ValueError(f'Expected YAML mapping: {path}')
    return value


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False))


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def resolve(base, value):
    p = Path(str(value)).expanduser()
    return p.resolve() if p.is_absolute() else (Path(base) / p).resolve()


def class_names(names):
    if isinstance(names, list):
        names = dict(enumerate(names))
    if not isinstance(names, dict) or not names:
        raise ValueError('names must be a nonempty list or contiguous integer mapping')
    names = {int(k): str(v) for k, v in names.items()}
    if sorted(names) != list(range(len(names))) or len(set(names.values())) != len(names):
        raise ValueError('Class IDs must be 0..N-1 and names must be unique')
    if any(not v.strip() for v in names.values()):
        raise ValueError('Empty class name')
    return names


def environment():
    versions = {}
    for name in ('torch', 'torchvision', 'ultralytics', 'numpy', 'opencv-python',
                 'onnx', 'onnxruntime'):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    result = dict(python=platform.python_version(), platform=platform.platform(),
                  packages=versions)
    try:
        result['git_commit'] = subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        result['git_commit'] = None
    return result


def new_output(path):
    path = Path(path).resolve()
    path.mkdir(parents=True, exist_ok=False)
    return path


def snapshot(folder, config, data, manifest):
    (folder / 'config.yaml').write_text(yaml.safe_dump(config, allow_unicode=True))
    (folder / 'data.yaml').write_text(yaml.safe_dump(data, allow_unicode=True))
    write_json(folder / 'dataset_manifest.json', manifest)
    write_json(folder / 'environment.json', environment())
    freeze = subprocess.check_output(
        [__import__('sys').executable, '-m', 'pip', 'freeze'], text=True)
    (folder / 'requirements.freeze.txt').write_text(freeze)


def check_model_names(model, names):
    actual = class_names(model.names)
    if actual != class_names(names):
        raise ValueError(f'Model/data class mismatch: model={actual}, data={names}')
    return actual
