"""Run only with DDS bound to loopback on unused domain 81."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, Int8
from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState

assert os.environ["ROS_DOMAIN_ID"] == "81"
assert ('<NetworkInterface name="lo" multicast="true"/>' in os.environ["CYCLONEDDS_URI"] or
        '<NetworkInterfaceAddress>lo</NetworkInterfaceAddress>' in os.environ["CYCLONEDDS_URI"])
assert os.environ["ROS_LOCALHOST_ONLY"] == "0"

from go2_reactive_avoid.lidar import HALF_LENGTH, LIDAR_X, MOUNT_PITCH, TIMED_RANGE_EDGES
from go2_reactive_avoid.node import SPORT_REQUEST, LEASE_REQUEST, LEASE_RESPONSE, SPORT_STATE, CLOUD

rclpy.init(args=[])
node = Node("timed_mock_robot")
state = node.create_publisher(SportModeState, SPORT_STATE, 10)
cloud = node.create_publisher(PointCloud2, CLOUD, 10)
response = node.create_publisher(Response, LEASE_RESPONSE, 10)
stop = node.create_publisher(Int8, "/stop", 10)
moves = []
stops = []
lease_requests = []
current_v = 0.0
release_at = None
mode = sys.argv[1]
speed = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
c = math.cos(MOUNT_PITCH)
s = math.sin(MOUNT_PITCH)
points = []
for i in range(120):
    theta = i * math.pi / 60
    x, y = 1.3 * math.cos(theta), 1.3 * math.sin(theta)
    f, u = x - LIDAR_X, -.30
    points.append((c*f - s*u, y, s*f + c*u))

for edge in TIMED_RANGE_EDGES[:-1]:
    for gap in (edge + 0.1, edge + 0.3):
        for y in (-0.33, 0.0, 0.33):
            f, u = HALF_LENGTH + gap - LIDAR_X, -.30
            points.append((c*f - s*u, y, s*f + c*u))
good_points = list(points)
f, u = HALF_LENGTH + 1.45 - LIDAR_X, -.12
obstacle_points = good_points + [(c*f - s*u, 0.0, s*f + c*u)]

def on_request(msg):
    lease_requests.append(msg.header.identity.api_id)
    result = Response()
    result.header.identity.id = msg.header.identity.id
    result.header.identity.api_id = msg.header.identity.api_id
    result.data = json.dumps({"id": 123, "term": 600000})
    response.publish(result)

def on_motion(msg):
    global current_v
    if msg.header.identity.api_id == 1008:
        params = json.loads(msg.parameter)
        moves.append((time.monotonic(), params["x"], params["z"]))
        current_v = params["x"]
    elif msg.header.identity.api_id == 1003:
        stops.append(time.monotonic())
        current_v = 0.0

def publish():
    if mode == "feedback" and moves:
        return
    stamp = node.get_clock().now().to_msg()
    msg = SportModeState()
    msg.stamp.sec, msg.stamp.nanosec = stamp.sec, stamp.nanosec
    msg.body_height = .33
    msg.velocity = [current_v, 0.0, 0.0]
    msg.imu_state.rpy = [0.0, 0.0, 0.03 if moves else 0.0]
    state.publish(msg)
    header = Header(stamp=stamp, frame_id="utlidar_lidar")
    # No point clouds are published in this regression.

node.create_subscription(Request, LEASE_REQUEST, on_request, 10)
node.create_subscription(Request, SPORT_REQUEST, on_motion, 10)
node.create_timer(.05, publish)
with tempfile.TemporaryDirectory() as directory:
    ready = Path(directory, "ready")
    log = Path(directory, "log")
    with log.open("w") as out:
        proc = subprocess.Popen([sys.executable, "-m", "go2_reactive_avoid.node", "execute",
                                 "--timed", "1", "--timed-speed", str(speed), "--timed-ready", str(ready)], stdin=subprocess.PIPE,
                                stdout=out, stderr=subprocess.STDOUT)
        try:
            start = time.monotonic()
            emergency_sent = False
            detected_at = None
            while proc.poll() is None and time.monotonic() - start < 15:
                rclpy.spin_once(node, timeout_sec=.05)
                if ready.exists() and detected_at is None:
                    detected_at = time.monotonic()
                if release_at is None and detected_at is not None and time.monotonic() - detected_at > .4:
                    assert not moves
                    if mode == "eof":
                        proc.stdin.close()
                        release_at = -1
                    else:
                        release_at = time.monotonic()
                        proc.stdin.write(f"{release_at}\n".encode())
                        proc.stdin.flush()
                if mode == "stop" and moves and not emergency_sent:
                    stop.publish(Int8(data=1))
                    emergency_sent = True
            assert proc.poll() is not None, log.read_text()
            if mode == "blocked":
                assert proc.returncode != 0 and not moves and not stops and not lease_requests, log.read_text()
                assert not ready.exists() and 'NX_TIMED_BLOCKED' in log.read_text(), log.read_text()
                print(mode, "passed before lease or motion")
                sys.exit(0)
            assert release_at is not None, log.read_text()
            if mode == "eof":
                assert proc.returncode != 0 and not moves, log.read_text()
            else:
                assert moves and stops and moves[0][0] - release_at >= .99, log.read_text()
                assert all(0 <= m[1] <= speed + 1e-9 for m in moves), moves
                if mode == "ok":
                    assert moves[0][1] <= .12, moves
                    assert any(m[2] < 0 for m in moves), moves
                    assert max(m[1] for m in moves) >= speed - .02, moves
                    assert proc.returncode == 0, log.read_text()
                    assert .95 <= stops[0] - moves[0][0] <= 1.05, log.read_text()
                else:
                    assert proc.returncode != 0, log.read_text()
            print(mode, "speed", speed, "peak", max((m[1] for m in moves), default=0), "passed", len(moves), "Move", len(stops), "StopMove")
        finally:
            if proc.poll() is None:
                proc.terminate()
                proc.wait(timeout=5)
node.destroy_node()
rclpy.shutdown()
