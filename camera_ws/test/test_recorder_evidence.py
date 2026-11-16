"""Run the evidence callback without camera, DDS, ROS node or video dependencies."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace


def evidence_callback():
    """Load the actual method only; constructing the recorder would connect DDS."""
    path = (Path(__file__).parents[1]/'src/go2_science_perception/'
            'go2_science_perception/approach_video_recorder.py')
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_evidence')
    namespace = {'json': json}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace['_evidence']


def test_evidence_preserves_exact_source_stamp_and_rejection_details():
    rows = []
    node = SimpleNamespace(_write=lambda kind, data: rows.append((kind, data)))
    callback = evidence_callback()
    evidence = dict(stamp_ns=1794836958612522949, reason='waiting_for_exact_source_tf',
                    tf_error='extrapolation into the past')
    callback(node, 'selection', SimpleNamespace(data=json.dumps(evidence)))
    assert rows == [('selection', {'selection': evidence})]
    callback(node, 'tracks', SimpleNamespace(data='{invalid'))
    assert rows[-1] == ('invalid_evidence', {'topic_kind': 'tracks', 'data': '{invalid'})
