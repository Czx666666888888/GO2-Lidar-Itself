"""Install the pinned 2026-11-18 S1 fusion delivery without replacing the v4 directory."""
import base64
import hashlib
from pathlib import Path
import zipfile

from s1_v2_compat import compatibility_payload, replace_once
from s1_v4_delivery import ensure_package, reset_queued_result, validate_archive

PACKAGE_VERSION = '20261118_latest_shared_grasp'
ARCHIVE_SHA256 = '8aee19a4f032c3f5a47a51c306a81e25ec7e05f6a17971f134022e2c443514f6'
ARCHIVE = 'camera_ws/s1_service/deliveries/S1_arm_fusion_latest_20261118.zip'
REMOTE_PACKAGE = '/home/unitree/s1-arm-control-20261118'
COMPAT_VERSION = 'fusion_20261118_status_1'


def delivery_payload(root):
    archive = Path(root) / ARCHIVE
    if hashlib.sha256(archive.read_bytes()).hexdigest() != ARCHIVE_SHA256:
        raise RuntimeError('S1 fusion archive hash mismatch; preserve')
    with zipfile.ZipFile(archive) as bundle:
        validate_archive(bundle, PACKAGE_VERSION)
        files = compatibility_payload(root, bundle, final_fusion=True,
                                      shared_grasp_version=PACKAGE_VERSION)
        # The integrated readiness check reads velocity as well as position and temperature.
        # Supply zero velocity in the upstream offline FakeArm; DumpArm inherits it.
        name = 'tests/test_arm_integration.py'
        original = bundle.read('s1-arm-control/' + name)
        content = replace_once(original.decode(), '    def get_temp(self):',
                               '    def get_vel(self): return [0.0]*7\n'
                               '    def get_temp(self):').encode()
        files[name] = dict(original_sha256=hashlib.sha256(original).hexdigest(),
                           content=base64.b64encode(content).decode(),
                           sha256=hashlib.sha256(content).hexdigest())
    reset_queued_result(files)
    return archive, dict(version=COMPAT_VERSION, archive_sha256=ARCHIVE_SHA256, files=files)


def ensure_fusion_package(root, ssh, host, *, running=False):
    return ensure_package(root, ssh, host, delivery_payload, REMOTE_PACKAGE, running=running)
