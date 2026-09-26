#!/bin/bash
# 快速测试 Point-LIO 在某个 bag 上是否发散 (回放前 60s, 记录 /state_estimation 轨迹)
# 用法: bash scripts/test_pointlio_bag.sh <bag目录> <输出npy路径>
source /home/lch/dog/scripts/env_autonomy.sh >/dev/null 2>&1
cd /home/lch/dog
BAG="$1"; OUT="$2"
LOG=/home/lch/dog/tmp/test_pointlio.log

ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false > "$LOG" 2>&1 &
LPID=$!
sleep 8

python3 - "$OUT" <<'PYEOF' > /home/lch/dog/tmp/test_pointlio_traj.log 2>&1 &
import sys, time
import numpy as np, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
OUT = sys.argv[1]
class C(Node):
    def __init__(self):
        super().__init__('t'); self.p=[]
        self.create_subscription(Odometry, '/state_estimation', self.cb, 10)
    def cb(self, m):
        p = m.pose.pose.position; self.p.append((p.x, p.y, p.z))
rclpy.init(); c = C()
t0 = time.time()
while time.time() - t0 < 55:
    rclpy.spin_once(c, timeout_sec=0.5)
rclpy.shutdown()
a = np.array(c.p)
np.save(OUT, a)
print('traj N=%d' % len(a))
if len(a):
    d = np.linalg.norm(a, axis=1)
    print('x:[%.2f,%.2f] y:[%.2f,%.2f] z:[%.2f,%.2f]' % (a[:,0].min(),a[:,0].max(),a[:,1].min(),a[:,1].max(),a[:,2].min(),a[:,2].max()))
    print('norm_max=%.1f diverged(>10m)=%d/%d' % (d.max(), (d>10).sum(), len(a)))
PYEOF
CPID=$!
sleep 2
timeout 58 ros2 bag play "$BAG" >/dev/null 2>&1
wait $CPID 2>/dev/null
kill $LPID 2>/dev/null
pkill -f pointlio_mapping 2>/dev/null; pkill -f transform_everything 2>/dev/null
cat /home/lch/dog/tmp/test_pointlio_traj.log
