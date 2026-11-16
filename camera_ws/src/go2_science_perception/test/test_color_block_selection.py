"""Only target selection/ambiguity boundaries; no robot or detector execution."""
from go2_science_perception.color_block_selection import BlockSelection


def test_nearest_is_latched_and_cannot_switch_to_another_visible_block():
    selector = BlockSelection()
    points = [(1.,0.,0.),(2.,0.,0.)]
    index,_ = selector.choose(points,(0.,0.))
    assert index == 0
    selector.commit(points[index])
    assert selector.choose([(2.,0.,0.)],(1.9,0.))[0] is None
    assert selector.choose([(2.,0.,0.),(1.02,0.,0.)],(1.9,0.))[0] == 1


def test_close_blocks_and_nonfinite_points_fail_closed():
    selector = BlockSelection()
    assert selector.choose([(1.,0.,0.),(1.15,0.,0.)],(0.,0.))[0] is None
    assert selector.choose([(float('nan'),0.,0.)],(0.,0.))[0] is None


def test_completed_round_unlocks_and_excludes_old_target_even_if_still_visible():
    selector = BlockSelection()
    selector.commit((1., 0., 0.))
    assert selector.choose([(2., 0., 0.)], (0., 0.))[0] is None
    assert selector.complete_round([[1., 0.]])
    # A completed block close to a new block no longer causes false ambiguity.
    points = [(1., 0., 0.), (1.3, 0., 0.), (2., 0., 0.)]
    index, _ = selector.choose(points, (0., 0.))
    assert index == 1
    selector.commit(points[index])
    assert not selector.complete_round([[1., 0.]])  # Duplicate acknowledgement is inert.
    assert selector.initial == points[index]
    assert selector.choose([(2., 0., 0.)], (0., 0.))[0] is None
    assert selector.complete_round([[1., 0.], [1.3, 0.]])
    assert selector.choose(points, (0., 0.))[0] == 2


def test_completed_only_and_ambiguous_remaining_targets_fail_closed():
    selector = BlockSelection()
    selector.complete_round([[1., 0.]])
    assert selector.choose([(1.01, 0., 0.)], (0., 0.))[1] == 'all_blocks_completed'
    assert selector.choose([(2., 0., 0.), (2.1, 0., 0.)], (0., 0.))[0] is None
    assert selector.choose([(1.151, 0., 0.)], (0., 0.))[0] == 0


def test_bad_or_nonappend_completion_preserves_current_lock():
    import pytest
    selector = BlockSelection()
    selector.complete_round([[1., 0.]])
    selector.commit((2., 0., 0.))
    for bad in [[], [[3., 0.]], [[1., 0.], [2., 0.], [3., 0.]],
                [[1., 0.], [float('nan'), 0.]], [[1., 0.], [True, 0.]], 'bad']:
        with pytest.raises(ValueError):
            selector.complete_round(bad)
        assert selector.initial == (2., 0., 0.)
        assert selector.completed == [(1., 0.)]
