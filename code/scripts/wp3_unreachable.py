#!/usr/bin/env python3
"""不可达目标测试: 把目标设在障碍密集处, 看 localPlanner 是否返回失败(1个航点)"""
import sys, time, math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from sensor_msgs_py import point_cloud2

class C(Node):
    def __init__(self):
        super().__init__("unreachable_test")
        self.obstacles = []
        self.latest_path = None
        self.create_subscription(PointCloud2, "/terrain_map", self.cb_terrain, 10)
        self.create_subscription(Path, "/path", self.cb_path, 10)
        self.goal_pub = self.create_publisher(PointStamped, "/way_point", 10)
    def cb_terrain(self, m):
        for p in point_cloud2.read_points(m, field_names=("x","y","z","intensity"), skip_nans=True):
            # 真实障碍: 0.3 < intensity < 1.45 (排除 1.5 的 unknown 注入点)
            if 0.3 < p[3] < 1.45:
                self.obstacles.append((p[0], p[1]))
    def cb_path(self, m):
        self.latest_path = m

rclpy.init(); c = C()
print("累积障碍...", flush=True)
start = time.time()
while time.time() - start < 30:
    rclpy.spin_once(c, timeout_sec=1.0)

if len(c.obstacles) < 20:
    print(f"障碍点太少({len(c.obstacles)}), 退出", flush=True)
    rclpy.shutdown(); sys.exit(1)

# 找障碍最密集处(限定合理范围), 作为"障碍内部"目标
ob = np.array(c.obstacles)
# 过滤离群点(限定 |x|,|y| < 6m)
ob = ob[(np.abs(ob[:,0]) < 6) & (np.abs(ob[:,1]) < 6)]
print(f"过滤后障碍点数={len(ob)}", flush=True)
if len(ob) < 20:
    print("过滤后障碍点太少, 退出", flush=True)
    rclpy.shutdown(); sys.exit(1)
# 找密度最高的网格块
grid = {}
for x, y in ob:
    k = (round(x*2), round(y*2))   # 0.5m 网格
    grid[k] = grid.get(k, 0) + 1
best_k = max(grid, key=grid.get)
gx, gy = best_k[0]/2.0, best_k[1]/2.0
print(f"障碍最密集网格=({gx:.2f}, {gy:.2f}), 网格内点数={grid[best_k]}", flush=True)
print(f"障碍范围: X[{ob[:,0].min():.2f},{ob[:,0].max():.2f}] Y[{ob[:,1].min():.2f},{ob[:,1].max():.2f}]", flush=True)

# 发布目标(设在障碍质心)
g = PointStamped()
g.header.frame_id = "camera_init"
g.point.x = gx; g.point.y = gy
c.goal_pub.publish(g)
print(f"已发布目标 ({gx:.2f}, {gy:.2f}) 到 /way_point", flush=True)

# 等 localPlanner 反应
time.sleep(6)
rclpy.spin_once(c, timeout_sec=1.0)

if c.latest_path is None:
    print("NO_PATH", flush=True)
else:
    n = len(c.latest_path.poses)
    print(f"最新 /path 航点数 = {n}", flush=True)
    if n <= 1:
        print("结果: ✅ 不可达目标 → 返回失败(空路径 1 航点)", flush=True)
    else:
        # 检查路径是否指向障碍(穿障)
        print(f"结果: ⚠️ 目标在障碍内但仍生成 {n} 航点路径(需检查是否穿障)", flush=True)
print("UNREACHABLE_DONE", flush=True)
rclpy.shutdown()
