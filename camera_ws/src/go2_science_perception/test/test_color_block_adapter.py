"""Exercise the ROS-message adapter without creating a node or connecting DDS."""
from types import SimpleNamespace as NS
from unittest.mock import Mock

from geometry_msgs.msg import TransformStamped

from go2_science_perception.color_block_selection import BlockSelection, CloseBlockFilter
from go2_science_perception.color_block_target_adapter import ColorBlockTargetAdapter
from go2_science_perception.science_target_manager import TargetTracker


def test_complete_frames_publish_one_latched_block_and_reject_invalid_depth():
    node = ColorBlockTargetAdapter.__new__(ColorBlockTargetAdapter)
    node.map_frame = 'map'
    node.selection = BlockSelection()
    node.last_published = 0
    node.input_count = 0
    node.get_clock = lambda: NS(now=lambda:NS(nanoseconds=1_100_000_000))
    node.get_parameter = lambda _: NS(value=.3)
    node.get_name = lambda:'camera_target_to_map'
    node.get_publishers_info_by_topic = lambda _:[NS(node_name=node.get_name())]
    node.count_publishers = lambda _:1
    tf = TransformStamped();tf.header.frame_id='map';tf.transform.rotation.w=1.
    node.tf_buffer = NS(lookup_transform=lambda *args:tf)
    node.camera_point = NS(publish=Mock())
    node.selection_debug = NS(publish=Mock())
    node._transform_and_publish = Mock()
    def detection(x,valid=True,cls=0):
        return dict(class_id=cls,class_name='color_block' if cls==0 else 'rock_spiky',
                    bbox_xyxy=[1,2,20,30],depth=dict(valid=valid),xyz_m=[x,0.,1.])
    def frame(stamp, detections):
        node.latest=dict(rgb_stamp_ns=stamp,status='OK',frame_id='camera_color_optical_frame',detections=detections)
        node._process()
    frame(1_000_000_000,[detection(2.),detection(1.),detection(.1,cls=2)])
    assert node.camera_point.publish.call_count==1
    point=node.camera_point.publish.call_args.args[0]
    assert point.point.x==1. and point.header.stamp.sec==1
    assert point.header.frame_id=='camera_color_optical_frame'
    frame(1_020_000_000,[detection(2.)])  # Locked target disappears; never switch.
    import json
    diagnostic = json.loads(node.selection_debug.publish.call_args.args[0].data)
    assert diagnostic['reason'] == 'locked_block_missing_or_ambiguous'
    assert diagnostic['published_count'] == 0
    assert diagnostic['association_checks'][0]['initial_rejected']
    assert diagnostic['association_checks'][0]['update_rejected']
    frame(1_040_000_000,[detection(1.,valid=False),detection(2.)])
    assert node.camera_point.publish.call_count==1
    frame(1_060_000_000,[detection(1.02)])
    assert node.camera_point.publish.call_count==2
    diagnostic = json.loads(node.selection_debug.publish.call_args.args[0].data)
    assert diagnostic['published_count'] == 1
    assert not diagnostic['association_checks'][0]['initial_rejected']
    # A failed TF query never masquerades as a spatial gate rejection or unlocks identity.
    from tf2_ros import TransformException
    node.tf_buffer.lookup_transform = Mock(side_effect=TransformException('source time absent'))
    frame(1_080_000_000,[detection(1.03)])
    diagnostic = json.loads(node.selection_debug.publish.call_args.args[0].data)
    assert diagnostic['reason'] == 'waiting_for_exact_source_tf'
    assert diagnostic['tf_error'] == 'source time absent'
    assert 'association_checks' not in diagnostic
    assert node.camera_point.publish.call_count == 2
    assert node.selection.current == (1.02, 0., 1.)


def test_round_reset_discards_queued_frame_and_rejects_replayed_old_rgb():
    import json
    from std_msgs.msg import String
    node = ColorBlockTargetAdapter.__new__(ColorBlockTargetAdapter)
    node.selection = BlockSelection()
    node.selection.commit((1., 0., 0.))
    node.latest = {'rgb_stamp_ns': 900}
    node.last_published = 800
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=1000))
    node.get_logger = lambda: NS(info=Mock())
    param = NS(name='completed_targets_json', value=json.dumps([[1., 0.]]))
    assert node._completed_round([param]).successful
    assert node.latest is None and node.last_published == 1000
    node._frame(String(data=json.dumps({'rgb_stamp_ns': 999})))
    assert node.latest is None
    node._frame(String(data=json.dumps({'rgb_stamp_ns': 1001})))
    assert node.latest['rgb_stamp_ns'] == 1001
    node.selection.commit((2., 0., 0.))
    assert node._completed_round([param]).successful
    assert node.selection.initial == (2., 0., 0.) and node.latest is not None
    param.value = '[]'
    assert not node._completed_round([param]).successful
    assert node.selection.initial == (2., 0., 0.)


def test_multi_mode_maps_all_valid_blocks_with_one_source_stamp():
    node = ColorBlockTargetAdapter.__new__(ColorBlockTargetAdapter)
    node.multi_target_memory = True
    node.close_blocks = CloseBlockFilter()
    node.map_frame, node.last_published, node.input_count = 'map', 0, 0
    node.selection = BlockSelection()
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=1_100_000_000))
    node.get_parameter = lambda _: NS(value=.3)
    node.get_name = lambda: 'camera_target_to_map'
    node.get_publishers_info_by_topic = lambda _: [NS(node_name=node.get_name())]
    node.count_publishers = lambda _: 1
    tf = TransformStamped();tf.transform.rotation.w = 1.
    node.tf_buffer = NS(lookup_transform=lambda *args: tf)
    node.camera_point, node.selection_debug = NS(publish=Mock()), NS(publish=Mock())
    node._transform_and_publish = Mock()
    def detection(x, valid):
        return dict(class_id=0,class_name='color_block',depth={'valid':valid},xyz_m=[x,0.,1.])
    node.latest = dict(rgb_stamp_ns=1_000_000_000,status='OK',frame_id='camera_color_optical_frame',
                       detections=[detection(1.,True),detection(2.,True),detection(3.,False)])
    node._process()
    assert node._transform_and_publish.call_count == 2
    points = [call.args[0] for call in node._transform_and_publish.call_args_list]
    assert [p.point.x for p in points] == [1.,2.]
    assert all(p.header.stamp.sec == 1 and p.header.stamp.nanosec == 0 for p in points)
    assert node.selection.initial is None and node.last_published == 1_000_000_000

    node.latest = dict(rgb_stamp_ns=1_020_000_000, status='OK',
                       frame_id='camera_color_optical_frame',
                       detections=[detection(1.04, True), detection(1.01, True)])
    node._process()
    assert node._transform_and_publish.call_count == 3
    assert node._transform_and_publish.call_args.args[0].point.x == 1.01


def test_isolated_pair_prefers_frame_support_then_confidence():
    """Keep the observation with the strongest history and break ties by confidence."""
    selector = CloseBlockFilter()
    points = [(0., 0.), (.04, 0.)]
    assert selector.update(10, points, [.3, .9]) == [1]
    assert selector.update(20, points[:1], [.3]) == [0]
    assert selector.update(30, points, [.3, .9]) == [0]
    assert selector.update(40, points[:1], [.3]) == [0]


def test_close_pair_needs_three_consecutive_distinct_frames():
    """Require consecutive source frames and reset after missing evidence."""
    selector = CloseBlockFilter()
    points = [(0., 0.), (.04, 0.)]
    assert selector.update(10, points, [.3, .9]) == [1]
    assert selector.update(10, points, [.3, .9]) == []
    assert selector.update(20, points[::-1], [.9, .3]) == [0]
    assert selector.update(30, points, [.3, .9]) == [0, 1]
    assert selector.update(40, [], []) == []
    assert selector.update(50, points, [.3, .9]) == [1]
    assert selector.update(60, points, [.3, .9]) == [1]
    assert selector.update(400_000_000, points, [.3, .9]) == [1]
    assert selector.update(500_000_000, [(0., 0.), (.10, 0.)], [.3, .9]) == [0, 1]


def test_persistent_close_pair_reaches_manager_as_two_tracks():
    """Preserve two tracks after the close pair is repeatedly observed."""
    selector = CloseBlockFilter()
    tracker = TargetTracker(.20, 5, 15, merge_close_tracks=False)
    points = [(0., 0., 0.), (.04, 0., 0.)]
    for stamp in range(1, 15):
        kept = selector.update(stamp, points, [.9, .8])
        tracker.update_frame([points[i] for i in kept], stamp)
    assert len(tracker.confirmed_unvisited()) == 2
