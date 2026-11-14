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
    frame(1_040_000_000,[detection(1.,valid=False),detection(2.)])
    assert node.camera_point.publish.call_count==1
    frame(1_060_000_000,[detection(1.02)])
    assert node.camera_point.publish.call_count==2
