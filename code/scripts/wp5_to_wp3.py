#!/usr/bin/env python3
"""①串闭环: WP5最优观察点 → /way_point → localPlanner → /path
验证: 路径存在 + 朝向观察点 + 无碰撞
"""
import sys, time, math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from sensor_msgs_py import point_cloud2

GX, GY = float(sys.argv[1]), float(sys.argv[2])  # WP5 最优观察点

class C(Node):
    def __init__(self):
        super().__init__("loop_check")
        self.path = None; self.odom = None; self.obstacles = []
        self.create_subscription(Path, "/path", self.cb_path, 10)
        self.create_subscription(Odometry, "/state_estimation", self.cb_odom, 10)
        self.create_subscription(PointCloud2, "/terrain_map", self.cb_terrain, 10)
        self.goal_pub = self.create_publisher(PointStamped, "/way_point", 10)
    def cb_path(self, m): self.path = m
    def cb_odom(self, m): self.odom = m
    def cb_terrain(self, m):
        for p in point_cloud2.read_points(m, field_names=("x","y","z","intensity"), skip_nans=True):
            if 0.3 < p[3] < 1.45:  # 真实障碍(排除1.5未知)
                self.obstacles.append((p[0], p[1], p[2]))

def yaw_from_quat(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

rclpy.init(); c = C()
print(f"WP5最优观察点目标=({GX:.2f},{GY:.2f})", flush=True)

# 等待数据(障碍累积 + 里程计 + 至少到轨迹后段)
start = time.time()
while time.time() - start < 45:
    rclpy.spin_once(c, timeout_sec=1.0)
    if c.odom is not None and len(c.obstacles) > 50:
        break

# 发布目标
g = PointStamped(); g.header.frame_id = "camera_init"
g.point.x = GX; g.point.y = GY; g.point.z = 0.0
c.goal_pub.publish(g)
print(f"已发布观察点到 /way_point", flush=True)

time.sleep(5)
rclpy.spin_once(c, timeout_sec=1.0)

if c.path is None:
    print("结果: ❌ localPlanner 未生成路径", flush=True)
elif c.odom is None:
    print("结果: ❌ 无里程计", flush=True)
else:
    n = len(c.path.poses)
    # 车辆位姿
    px, py = c.odom.pose.pose.position.x, c.odom.pose.pose.position.y
    yaw = yaw_from_quat(c.odom.pose.pose.orientation)
    # 路径航点(vehicle系→map系)
    path_pts = []
    for pose in c.path.poses:
        lx, ly = pose.pose.position.x, pose.pose.position.y
        mx = px + lx*math.cos(yaw) - ly*math.sin(yaw)
        my = py + lx*math.sin(yaw) + ly*math.cos(yaw)
        path_pts.append((mx, my))
    # 路径终点方向 vs 观察点方向
    if path_pts:
        end = path_pts[-1]
        dir_to_goal = math.atan2(GY-py, GX-px)
        dir_to_end = math.atan2(end[1]-py, end[0]-px)
        # 碰撞检查
        ob = np.array(c.obstacles)
        min_d = min(np.hypot(ob[:,0]-mx, ob[:,1]-my).min() for mx,my in path_pts) if len(ob) else 99
        coll_ok = min_d >= 0.45
        # 方向检查(路径终点是否朝目标方向)
        ang_diff = abs(math.degrees(dir_to_end - dir_to_goal))
        ang_diff = min(ang_diff, 360-ang_diff)
        print(f"结果: ✅ 生成 {n} 航点路径", flush=True)
        print(f"  车辆=({px:.2f},{py:.2f}) 观察点=({GX:.2f},{GY:.2f})", flush=True)
        print(f"  路径终点=({end[0]:.2f},{end[1]:.2f}) 朝向偏差={ang_diff:.1f}°", flush=True)
        print(f"  无碰撞净空={min_d:.2f}m {'✅' if coll_ok else '⚠️ 有碰撞'}", flush=True)
        print(f"  闭环判定: {'✅ 探索→规划闭环成立' if (n>1 and coll_ok and ang_diff<45) else '⚠️ 需检查'}", flush=True)

print("LOOP_DONE", flush=True)
rclpy.shutdown()
