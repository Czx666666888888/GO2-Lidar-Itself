#!/usr/bin/env python3
"""检测 far-vs-local 死锁: waypoint 长时间不变 + path 空/短 + 速度0, 且狗不在 waypoint 附近
用法: python3 detect_far_local_deadlock.py <bag1> [bag2 ...]
"""
import sys
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PointStamped, TwistStamped
from nav_msgs.msg import Odometry, Path

def analyze(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'),
           ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
    way=[]; pathn=[]; cmd=[]; traj=[]; goals=[]
    while r.has_next():
        topic,data,t = r.read_next()
        ts = t/1e9
        if topic=='/way_point':
            m=deserialize_message(data,PointStamped); way.append((ts,m.point.x,m.point.y))
        elif topic=='/path':
            m=deserialize_message(data,Path); pathn.append((ts,len(m.poses)))
        elif topic=='/cmd_vel_raw':
            m=deserialize_message(data,TwistStamped); cmd.append((ts,m.twist.linear.x))
        elif topic=='/state_estimation':
            m=deserialize_message(data,Odometry); traj.append((ts,m.pose.pose.position.x,m.pose.pose.position.y))
        elif topic=='/goal_point':
            m=deserialize_message(data,PointStamped); goals.append((ts,m.point.x,m.point.y))

    name=bag.split('/')[-1]
    print(f"\n===== {name} =====")
    if not way or not pathn or not cmd or not traj:
        print("  缺话题, 跳过"); return

    way_t=np.array([t for t,_,_ in way]); way_p=np.array([(x,y) for _,x,y in way])
    cmd_t=np.array([t for t,_ in cmd]); cmd_v=np.array([v for _,v in cmd])
    tr_t=np.array([t for t,_,_ in traj]); tr_p=np.array([(x,y) for _,x,y in traj])

    # 找"停滞段": 狗速度<0.01 连续>5s
    # 用 cmd_vel 判断
    stuck=[]; s=None
    for i,(t,v) in enumerate(zip(cmd_t,cmd_v)):
        moving = abs(v) > 0.01
        if not moving and s is None: s=i
        elif moving and s is not None:
            if cmd_t[i-1]-cmd_t[s] >= 5: stuck.append((cmd_t[s], cmd_t[i-1]))
            s=None
    if s is not None and cmd_t[-1]-cmd_t[s]>=5: stuck.append((cmd_t[s],cmd_t[-1]))

    if not stuck:
        print("  无 >5s 的静止段"); return

    print(f"  检测到 {len(stuck)} 个静止段(>5s):")
    for (t0,t1) in stuck:
        # 该段内 waypoint 是否固定
        wm = way_t[(way_t>=t0)&(way_t<=t1)]
        wp = way_p[(way_t>=t0)&(way_t<=t1)]
        w_fixed = False
        if len(wp)>=1:
            w_fixed = (np.ptp(wp[:,0])<0.05 and np.ptp(wp[:,1])<0.05)
        # 该段内 path 点数
        pn = [n for t,n in pathn if t0<=t<=t1]
        p_empty = (len(pn)==0 or np.mean(pn)<2) if pn else True
        # 该段内狗到 waypoint 的距离
        j=np.searchsorted(tr_t, (t0+t1)/2); j=min(j,len(tr_t)-1)
        dogpos=tr_p[j]
        wdist = np.hypot(wp[-1,0]-dogpos[0], wp[-1,1]-dogpos[1]) if len(wp) else -1
        # goal 是真实目标还是兜底
        gx = [g for g in goals if t0<=g[0]<=t1]
        flag = ""
        if w_fixed and p_empty:
            flag = "★ far-vs-local 死锁(waypoint固定+path空)"
        elif w_fixed and wdist>0.3:
            flag = "△ waypoint固定且狗离它>0.3m(path有但走不到?)"
        elif w_fixed:
            flag = "waypoint固定(但狗已在附近)"
        else:
            flag = "waypoint在变(非此问题)"
        print(f"    t={t0:6.1f}~{t1:6.1f}s ({t1-t0:4.0f}s) waypoint_fixed={w_fixed} path_empty={p_empty} 狗距waypoint={wdist:.2f}m  {flag}")

for b in sys.argv[1:]:
    analyze(b)
