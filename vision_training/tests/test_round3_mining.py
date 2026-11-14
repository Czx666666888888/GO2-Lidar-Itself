"""Mining hints never establish target absence or true rock identity."""
from vision_training.tools.prepare_round3 import hints,box_iou


def test_no_predictions_are_not_negatives():
    flags=hints(dict(predictions=[],blur_metric=2,motion_mae=4))
    assert 'v2_prediction_absent_not_negative' in flags
    assert 'low_laplacian_review_only' in flags
    assert 'negative' not in flags


def test_predicted_size_edge_and_class_change_remain_hints():
    flags=hints(dict(predictions=[dict(class_id=1,short_side_at_640=12,edge_candidate=True)],
                     predicted_class_change=True,control_log=dict(state='ALIGN',wz=.2)))
    assert 'predicted_small_lt16' in flags and 'predicted_edge' in flags
    assert 'control_motion_hint' in flags and 'predicted_class_change' in flags
    assert box_iou([0,0,10,10],[0,0,10,10])==1
    assert box_iou([0,0,10,10],[20,20,30,30])==0
