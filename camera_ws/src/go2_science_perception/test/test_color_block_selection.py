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
