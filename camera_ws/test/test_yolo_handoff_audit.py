"""Regression for absent evidence and retry/frame count distinctions."""
import importlib.util
import json
from pathlib import Path


SPEC = importlib.util.spec_from_file_location(
    'audit_yolo_handoff', Path(__file__).parents[1]/'scripts/audit_yolo_handoff.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_old_run_does_not_report_unrecorded_tf_or_lock_failures_as_zero(tmp_path):
    rows = [dict(kind='detection', detection=dict(color_stamp_ns=10, targets=[
        dict(accepted=True, depth=dict(reason='accepted_central_roi'))]))]
    (tmp_path/'approach_one.jsonl').write_text('\n'.join(map(json.dumps, rows)))
    (tmp_path/'slam.log').write_text('FINAL_MAP_TRANSFORM input=0 transformed=0 failed=0')
    report = MODULE.audit(tmp_path)
    assert report['valid_depth_frames'] == 1
    assert report['adapter_final']['transformed'] == 0
    assert report['selection_reasons'] is None
    assert report['initial_gate_rejections'] is None
    assert report['exact_tf_failed_attempts'] is None


def test_retries_distinct_from_unique_frames_and_gates_can_both_fail(tmp_path):
    failure = dict(stamp_ns=10, reason='waiting_for_exact_source_tf')
    locked = dict(stamp_ns=20, reason='locked_block_missing_or_ambiguous',
                  association_checks=[dict(initial_rejected=True, update_rejected=True)])
    rows = [dict(kind='selection', selection=s) for s in (failure, failure, locked)]
    (tmp_path/'approach_one.jsonl').write_text('\n'.join(map(json.dumps, rows)))
    report = MODULE.audit(tmp_path)
    assert report['exact_tf_failed_attempts'] == 2
    assert report['selection_unique_frames_by_reason']['waiting_for_exact_source_tf'] == 1
    assert report['initial_gate_rejections'] == report['update_gate_rejections'] == 1
