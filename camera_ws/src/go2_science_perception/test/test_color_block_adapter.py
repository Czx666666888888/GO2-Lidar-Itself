"""Exercise the ROS-message adapter without creating a node or connecting DDS."""
from types import SimpleNamespace as NS
from unittest.mock import Mock

from geometry_msgs.msg import TransformStamped

from go2_science_perception.color_block_selection import BlockSelection
from go2_science_perception.color_block_target_adapter import ColorBlockTargetAdapter


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
