#!/usr/bin/env python3
"""Read-only event recorder. Preserve raw events; never substitute latest TF/odom."""
import argparse
import json
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', required=True, choices=['A', 'B', 'C'])
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('map_drift_recorder')
    with open(args.output, 'x', buffering=1) as stream:
        def write(kind, data):
            stream.write(json.dumps(dict(kind=kind, label=args.label,
                                         received_ns=node.get_clock().now().nanoseconds,
                                         data=data), allow_nan=False) + '\n')

        for topic, kind in [('/science/detection_debug', 'detection'),
                            ('/science/map_measurement_debug', 'measurement'),
                            ('/science/target_tracks', 'tracks')]:
            node.create_subscription(String, topic,
                                     lambda msg, k=kind: write(k, json.loads(msg.data)), 100)
        node.create_subscription(Odometry, '/state_estimation',
                                 lambda msg: write('point_lio', message_to_ordereddict(msg)),
                                 qos_profile_sensor_data)
        deadline = time.monotonic() + args.seconds
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
