"""Check GT matching so background and duplicate boxes cannot inflate success."""
import numpy as np

from vision_training.tools.analyze_pilot import ious, matched_counts


def test_iou_and_single_gt_duplicate_matching():
    gt = np.array([10, 10, 20, 20])
    boxes = [[10, 10, 20, 20], [10, 10, 20, 20], [30, 30, 40, 40]]
    assert np.allclose(ious(boxes, gt), [1, 1, 0])
    counts = matched_counts(boxes, [.9, .8, .7], [0, 0, 0], gt, .05)
    assert (counts['tp'], counts['fp'], counts['fn']) == (1, 2, 0)
    assert counts['background_iou_below_01'] == 1


def test_wrong_class_and_poor_localization_do_not_match():
    gt = np.array([10, 10, 20, 20])
    counts = matched_counts([[10, 10, 20, 20], [15, 10, 25, 20]], [.9, .8], [1, 0], gt, .1)
    assert (counts['tp'], counts['fp'], counts['fn']) == (0, 2, 1)
    assert counts['wrong_class'] == 1
    assert counts['localization_iou_01_to_05'] == 1


def test_empty_threshold_output_keeps_missing_gt_and_null_iou():
    counts = matched_counts([], [], [], np.array([10, 10, 20, 20]), .01)
    assert (counts['predictions'], counts['tp'], counts['fp'], counts['fn']) == (0, 0, 0, 1)
    assert counts['best_iou'] is None
    assert counts['matched_confidence'] is None
