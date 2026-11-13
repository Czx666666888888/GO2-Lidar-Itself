#!/usr/bin/env python3
"""Bounded read-only startup/reuse check. Exits as soon as real data is ready."""
import argparse
from collections import defaultdict, deque
import json
import math
import os
from pathlib import Path
import statistics
import time


def probe(timeout=8., scope='all', require_target=False):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, Imu, Image
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from geometry_msgs.msg import PointStamped
    from rcl_interfaces.srv import GetParameters
    from unitree_go.msg import LidarState
    from tf2_ros import Buffer, TransformListener
    from go2_time_diagnostic import cloud_times
    rclpy.init()
    n = rclpy.create_node('go2_startup_readonly')
    samples = defaultdict(lambda: deque(maxlen=500))
    mapping = {}; tracks = {}; native = {}; bad = []
    tf = Buffer(); listener = TransformListener(tf, n)
    begin = time.monotonic(); baseline = time.time()-begin
    parameter_client=n.create_client(GetParameters,'/laserMapping/get_parameters')
    parameter_future=None
    selected_position=[]
    def receive(m, key):
        now = time.monotonic()
        if key in ('mapping', 'tracks'):
            try:
                dest = mapping if key == 'mapping' else tracks
                dest.clear(); dest.update(value=json.loads(m.data), received=now)
            except ValueError:
                bad.append('invalid '+key+' JSON')
            return
        if key == 'native':
            native.update(hz=float(m.cloud_frequency), loss=float(m.cloud_packet_loss_rate),
                          received=now)
            return
        stamp = m.header.stamp.sec+m.header.stamp.nanosec/1e9
        if samples[key] and key not in ('rgb', 'depth') and stamp <= samples[key][-1][1]:
            bad.append(key+' timestamp not advancing')
        if key == 'selected':
            selected_position[:]=[m.point.x,m.point.y,m.point.z]
        extra = cloud_times(m) if key == 'raw_cloud' else {}
        if key in ('odom', 'base'):
            p=m.pose.pose.position
            if not all(math.isfinite(x) for x in (p.x,p.y,p.z)):
                bad.append(key+' nonfinite pose')
        samples[key].append((now,stamp,extra))
    specs = ([('raw_cloud','/utlidar/cloud',PointCloud2), ('raw_imu','/utlidar/imu',Imu),
              ('native','/utlidar/lidar_state',LidarState)] if scope!='chain' else [])
    if scope != 'native':
        specs += [('cloud','/utlidar/transformed_cloud',PointCloud2),
                  ('imu','/utlidar/transformed_imu',Imu), ('odom','/state_estimation',Odometry),
                  ('base','/base_state_estimation',Odometry),
                  ('rgb','/camera/camera/color/image_raw',Image),
                  ('depth','/camera/camera/aligned_depth_to_color/image_raw',Image),
                  ('mapping','/utlidar/time_mapping/status',String),
                  ('tracks','/science/target_tracks',String),
                  ('selected','/science/selected_target',PointStamped)]
    subscriptions=[n.create_subscription(typ,topic,lambda m,key=key:receive(m,key),
                                         qos_profile_sensor_data) for key,topic,typ in specs]
    report = {}
    def fresh(key, tolerance=.4, future=.15):
        rows=samples[key]
        return (len(rows)>=5 and rows[-1][0]-rows[0][0]>=.3 and
                time.monotonic()-rows[-1][0]<tolerance and
                -future <= time.time()-rows[-1][1] < tolerance)
    try:
        while time.monotonic()-begin < timeout:
            rclpy.spin_once(n,timeout_sec=.03)
            now=time.monotonic()
            if scope!='native' and parameter_future is None and parameter_client.service_is_ready():
                parameter_future=parameter_client.call_async(GetParameters.Request(names=['use_sim_time']))
            wall_clock=False
            if parameter_future is not None and parameter_future.done():
                try:
                    value=parameter_future.result().values[0]
                    wall_clock=value.type==1 and value.bool_value is False
                except (IndexError,AttributeError,RuntimeError):
                    pass
            if abs((time.time()-now)-baseline)>.1:
                bad.append('local wall clock jump')
            clouds=samples['raw_cloud']
            hz=((len(clouds)-1)/(clouds[-1][0]-clouds[0][0]) if len(clouds)>1 else 0.)
            spans=[r[2].get('point_time_max_s',999) for r in clouds]
            native_ok=(len(clouds)>=8 and clouds[-1][0]-clouds[0][0]>=.5 and
                       hz>=10 and native.get('hz',0)>=10 and native.get('loss',101)<=10 and
                       now-native.get('received',0)<1 and now-clouds[-1][0]<.2 and
                       statistics.median(spans)<=.1 and len(samples['raw_imu'])>=50 and
                       now-samples['raw_imu'][-1][0]<.1)
            camera_ok=fresh('rgb',.4,.05) and fresh('depth',.4,.05)
            mapped=mapping.get('value',{})
            mapping_ok=(mapped.get('ready') is True and not mapped.get('fault') and
                        mapped.get('mode')==os.environ.get('GO2_TIME_MAPPING_MODE','imu') and
                        now-mapping.get('received',0)<2.5 and
                        abs(time.time()-mapped.get('ros_stamp_ns',0)/1e9)<2.5)
            body_tf=camera_tf=False
            try:
                if fresh('base'):
                    tf.lookup_transform('map','vehicle',rclpy.time.Time(
                        nanoseconds=round(samples['base'][-1][1]*1e9)))
                    body_tf=True
                if camera_ok:
                    tf.lookup_transform('map','camera_link',rclpy.time.Time(
                        nanoseconds=round(samples['rgb'][-1][1]*1e9)))
                    camera_tf=True
            except Exception:
                pass
            slam_ok=all(fresh(k) for k in ('cloud','imu','odom','base')) and mapping_ok and body_tf and wall_clock and not bad
            target_ok=False
            ts=tracks.get('value',{})
            # Existing manager JSON contains active confirmed tracks, each with observed_ns.
            items=ts.get('tracks',[]) if isinstance(ts,dict) else ts
            for item in items if isinstance(items,list) else []:
                if (item.get('state')=='CONFIRMED' and not item.get('visited') and
                        0 <= time.time()-item.get('observed_ns',0)/1e9 < .5 and
                        fresh('selected',.5) and selected_position and
                        math.dist(selected_position,item.get('position',[math.inf]*3))<1e-5):
                    target_ok=True
            if n.count_publishers('/clock'):
                bad.append('unexpected /clock publisher')
            if n.count_publishers('/cmd_vel_raw'):
                bad.append('existing velocity controller publisher')
            report=dict(native_ok=native_ok if scope!='chain' else None, native_check_reused=scope=='chain',
                        checked_wall_ns=time.time_ns(),
                        base_stamp_ns=round(samples['base'][-1][1]*1e9) if samples['base'] else None,
                        camera_stamp_ns=round(samples['rgb'][-1][1]*1e9) if samples['rgb'] else None, native_received_hz=hz, native_state=native,
                        camera_ok=camera_ok, slam_ok=slam_ok, mapping=mapped,
                        body_tf=body_tf,camera_tf=camera_tf,target_ok=target_ok,use_sim_time_false=wall_clock,
                        errors=list(dict.fromkeys(bad)), sample_counts={k:len(v) for k,v in samples.items()},
                        elapsed_sec=now-begin)
            report['ready']=((native_ok or scope=='chain') and not bad and (scope=='native' or
                (slam_ok and camera_ok and camera_tf and (target_ok or not require_target))))
            if report['ready'] or ((scope=='chain' or native_ok) and
                    (bad or mapped.get('fault'))):
                break
        return report
    finally:
        n.destroy_node();rclpy.shutdown()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--timeout',type=float,default=8)
    p.add_argument('--scope',choices=['native','all','chain'],default='all')
    p.add_argument('--require-target',action='store_true')
    p.add_argument('--report',type=Path,required=True)
    a=p.parse_args()
    if not 0<a.timeout<=60:p.error('timeout must be in (0,60]')
    result=probe(a.timeout,a.scope,a.require_target)
    a.report.write_text(json.dumps(result,indent=2)+'\n')
    print('STARTUP_PROBE '+json.dumps(result),flush=True)
    return 0 if result.get('ready') else 2

if __name__=='__main__':
    raise SystemExit(main())
