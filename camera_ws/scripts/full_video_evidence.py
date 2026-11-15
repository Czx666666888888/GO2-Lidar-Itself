"""Index the existing front camera videos without touching the wrist camera."""
import json
from pathlib import Path


def save_front_evidence(folder, front):
    """Keep front evidence; D455 recording was removed at the user's request."""
    report = dict(front=front, wrist=dict(status='DISABLED', reason='D455 recording removed'))
    (Path(folder)/'recordings.json').write_text(json.dumps(report, indent=2)+'\n')
