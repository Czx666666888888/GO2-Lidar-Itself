"""Front recording remains independent from wrist acquisition and grasp."""
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'camera_ws/scripts'))
from full_video_evidence import save_front_evidence


def test_front_index_explicitly_disables_wrist_recording(tmp_path):
    front = dict(video='overlay.mp4', raw_video='raw.mp4', frames=20, closed=True)
    save_front_evidence(tmp_path, front)
    report = json.loads((tmp_path/'recordings.json').read_text())
    assert report['front'] == front
    assert report['wrist']['status'] == 'DISABLED'
