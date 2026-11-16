#!/usr/bin/env python3
"""Acknowledge one completed grasp/StandUp to the existing YOLO adapter."""
import json
import sys


def main(argv=None):
    """Use the node's existing atomic parameter service; never publish motion."""
    import rclpy
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import SetParametersAtomically

    args = sys.argv[1:] if argv is None else argv
    if len(args) != 1 or not isinstance(json.loads(args[0]), list):
        raise ValueError('Expected completed MAP XY list')
    rclpy.init()
    node = rclpy.create_node('advance_yolo_completed_round')
    try:
        client = node.create_client(SetParametersAtomically,
                                   '/camera_target_to_map/set_parameters_atomically')
        if not client.wait_for_service(timeout_sec=3.):
            raise RuntimeError('YOLO round reset service unavailable; no next SEARCH')
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter('completed_targets_json', value=args[0]).to_parameter_msg()]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
        if not future.done() or future.exception() is not None:
            raise RuntimeError('YOLO round reset not acknowledged; no next SEARCH')
        result = future.result().result
        if not result.successful:
            raise RuntimeError('YOLO round reset rejected: '+result.reason)
        print('YOLO_ROUND_RESET_ACK '+args[0], flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
