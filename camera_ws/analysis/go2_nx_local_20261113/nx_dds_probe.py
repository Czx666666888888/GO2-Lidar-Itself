import rclpy,time,json
from sensor_msgs.msg import PointCloud2,Imu
from rclpy.qos import qos_profile_sensor_data
rclpy.init();n=rclpy.create_node('nx_readonly_native_probe');data={'cloud':[],'imu':[]}
def cb(kind,msg):
 data[kind].append((time.monotonic(),n.get_clock().now().nanoseconds,msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))
subs=[n.create_subscription(t,topic,lambda m,k=k:cb(k,m),qos_profile_sensor_data) for k,t,topic in [('cloud',PointCloud2,'/utlidar/cloud'),('imu',Imu,'/utlidar/imu')]]
end=time.monotonic()+12
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
print(json.dumps({k:{'count':len(v),'hz':(len(v)-1)/(v[-1][0]-v[0][0]) if len(v)>1 else None,'receive_minus_header_s':(v[-1][1]-v[-1][2])/1e9 if v else None} for k,v in data.items()}))
n.destroy_node();rclpy.shutdown()
