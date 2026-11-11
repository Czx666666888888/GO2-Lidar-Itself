#!/usr/bin/env python3
"""Exercise real YOLO/ROS with synthetic depth on an isolated loopback DDS domain."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', required=True)
    parser.add_argument('--image', required=True, help='image with known pretrained detections')
    parser.add_argument('--output', required=True)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--classes', default='', help='optional checkpoint names to select')
    args = parser.parse_args()
    # Do not connect to the robot, even if shell has the project's live DDS environment.
    os.environ['ROS_DOMAIN_ID'] = '78'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    os.environ['CYCLONEDDS_URI'] = (
        '<CycloneDDS><Domain><General><NetworkInterfaceAddress>lo</NetworkInterfaceAddress>'
        '<AllowMulticast>false</AllowMulticast></General><Discovery><Peers>'
        '<Peer Address="127.0.0.1"/></Peers><ParticipantIndex>auto</ParticipantIndex>'
        '</Discovery></Domain></CycloneDDS>')
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import PointStamped
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('yolo_isolated_smoke')
    bridge = CvBridge()
    bgr = cv2.imread(args.image)
    if bgr is None:
        raise ValueError('Cannot read image')
    h, w = bgr.shape[:2]
    reports, points, debug_stamps = [], [], []
    node.create_subscription(String, '/science/yolo/detections',
                             lambda m: reports.append(json.loads(m.data)), 10)
    node.create_subscription(PointStamped, '/science/yolo_target_point',
                             lambda m: points.append(m), 10)
    node.create_subscription(Image, '/science/yolo/debug_image',
                             lambda m: debug_stamps.append(m.header.stamp.sec*10**9 +
                                                           m.header.stamp.nanosec), 10)
    rgb_pub = node.create_publisher(Image, '/yolo_test/color', qos_profile_sensor_data)
    depth_pub = node.create_publisher(Image, '/yolo_test/depth', qos_profile_sensor_data)
    info_pub = node.create_publisher(CameraInfo, '/yolo_test/info', qos_profile_sensor_data)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    log = output.with_suffix('.node.log').open('w')
    command = [sys.executable, '-m', 'go2_science_perception.yolo_detector', '--ros-args',
               '-p', f'model_path:={Path(args.weights).resolve()}', '-p', f'device:={args.device}',
               '-p', 'color_topic:=/yolo_test/color', '-p', 'depth_topic:=/yolo_test/depth',
               '-p', 'camera_info_topic:=/yolo_test/info', '-p', 'max_image_age_sec:=5.0']
    if args.classes:
        command += ['-p', f'class_names:={args.classes}']
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
    outcomes = []
    try:
        for case in ('valid_depth', 'invalid_depth', 'time_mismatch', 'frame_mismatch', 'no_depth'):
            end, result = time.monotonic()+35, None
            while time.monotonic() < end:
                if process.poll() is not None:
                    raise RuntimeError(f'YOLO exited; inspect {output.with_suffix(".node.log")}')
                stamp = node.get_clock().now().to_msg()
                stamp_id = stamp.sec*10**9+stamp.nanosec
                rgb = bridge.cv2_to_imgmsg(bgr, 'bgr8')
                rgb.header.stamp, rgb.header.frame_id = stamp, 'test_color_optical'
                info = CameraInfo()
                info.header, info.width, info.height = rgb.header, w, h
                info.k = [600.0, 0.0, w/2, 0.0, 600.0, h/2, 0.0, 0.0, 1.0]
                info.distortion_model = 'plumb_bob'
                info_pub.publish(info)
                if case != 'no_depth':
                    value = 0.0 if case == 'invalid_depth' else 1.2
                    depth = bridge.cv2_to_imgmsg(np.full((h, w), value, np.float32), '32FC1')
                    depth.header.stamp, depth.header.frame_id = stamp, 'test_color_optical'
                    if case == 'time_mismatch':
                        depth.header.stamp = rclpy.time.Time(
                            nanoseconds=stamp_id-500_000_000).to_msg()
                    if case == 'frame_mismatch':
                        depth.header.frame_id = 'wrong_optical'
                    depth_pub.publish(depth)
                rgb_pub.publish(rgb)
                settle = time.monotonic()+0.7
                while time.monotonic() < settle:
                    rclpy.spin_once(node, timeout_sec=.02)
                matching = [r for r in reports if r['rgb_stamp_ns'] == stamp_id]
                if matching and matching[-1]['detections']:
                    candidate = matching[-1]
                    has_xyz = any('xyz_m' in d for d in candidate['detections'])
                    if has_xyz == (case == 'valid_depth'):
                        result = candidate
                        break
            if result is None:
                raise AssertionError(f'No expected result for {case}')
            assert result['frame_id'] == 'test_color_optical'
            if args.classes:
                allowed = {name.strip() for name in args.classes.split(',')}
                assert {d['class_name'] for d in result['detections']} <= allowed
            assert result['rgb_stamp_ns'] in debug_stamps
            if case == 'valid_depth':
                assert any(p.header.stamp.sec*10**9+p.header.stamp.nanosec ==
                           result['rgb_stamp_ns'] for p in points)
            else:
                assert not any(p.header.stamp.sec*10**9+p.header.stamp.nanosec ==
                               result['rgb_stamp_ns'] for p in points)
            outcomes.append(dict(case=case, report=result))
        assert not node.get_publishers_info_by_topic('/science/target_coarse_point')
        assert not node.get_publishers_info_by_topic('/cmd_vel_raw')
        output.write_text(json.dumps(dict(result='PASS', domain=78, synthetic_depth=True,
                                          motion_publishers=[], cases=outcomes), indent=2))
        print(f'PASS: {len(outcomes)} real-model ROS cases; {output}')
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5)
        log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
