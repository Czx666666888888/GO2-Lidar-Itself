#!/usr/bin/env python
import rclpy
from rclpy.executors import SingleThreadedExecutor, await_or_execute
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped, Vector3
from std_msgs.msg import String
import sensor_msgs_py.point_cloud2 as pc2
import tf_transformations

from transforms3d.quaternions import quat2mat

from copy import deepcopy
import numpy as np
import yaml

import os
import json
import time

from transform_sensors.imu_time_mapping import ImuTimeMapping


class TimestampExecutor(SingleThreadedExecutor):
    """Reuse the diagnostic's MessageInfo-preserving adapter for installed Humble."""
    def _take_subscription(self, sub):
        with sub.handle:
            result = sub.handle.take_message(sub.msg_type, sub.raw)
        if result is None:
            return None
        return result if getattr(sub, 'time_probe', False) else (result[0],)

    async def _execute_subscription(self, sub, values):
        if values:
            await await_or_execute(sub.callback, *values)

class Repuber(Node):
    def __init__(self):
        super().__init__('sensor_transformer')
        # GO2 publishes its onboard UTLidar streams directly over DDS.  Use the
        # sensor-data profile so this subscriber is compatible with both the
        # robot's best-effort publisher and reliable rosbag playback.
        self.imu_sub = self.create_subscription(
            Imu, '/utlidar/imu', self.imu_callback, qos_profile_sensor_data)
        self.imu_sub.time_probe = True
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/utlidar/cloud', self.cloud_callback,
            qos_profile_sensor_data)
        
        self.imu_raw_pub = self.create_publisher(Imu, '/utlidar/transformed_raw_imu', 50)
        self.imu_pub = self.create_publisher(Imu, '/utlidar/transformed_imu', 50)
        self.cloud_pub = self.create_publisher(PointCloud2, '/utlidar/transformed_cloud', 50)

        self.imu_stationary_list = []
        
        # Restore a8f7c667's first-cloud shared offset without reverting controls.
        self.declare_parameter('time_mapping_mode', os.environ.get('GO2_TIME_MAPPING_MODE', 'legacy'))
        self.declare_parameter('sensor_time_offset_sec', 0.0)
        self.declare_parameter('time_mapping_window_sec', 2.0)
        self.declare_parameter('time_mapping_min_samples', 200)
        self.declare_parameter('time_mapping_percentile', 5.0)
        self.time_mode = self.get_parameter('time_mapping_mode').value
        if self.time_mode not in ('legacy', 'imu', 'explicit'):
            raise ValueError('time_mapping_mode must be legacy, imu or explicit')
        self.time_stamp_offset = 0
        self.time_stamp_offset_set = False
        self.mapping = None if self.time_mode == 'legacy' else ImuTimeMapping(
            window_sec=self.get_parameter('time_mapping_window_sec').value,
            min_samples=self.get_parameter('time_mapping_min_samples').value,
            percentile=self.get_parameter('time_mapping_percentile').value,
            explicit_offset_sec=(self.get_parameter('sensor_time_offset_sec').value
                                 if self.time_mode == 'explicit' else None))
        self.get_logger().warning(
            f'TIME_MODE={self.time_mode}; legacy uses first-cloud receipt minus header, '
            'not precise clock synchronization; historical odometry future bias about 57 ms')
        self.mapping_status_pub = self.create_publisher(String, '/utlidar/time_mapping/status', 10)
        self.create_timer(1.0, self.publish_mapping_status)
        
        self.cam_offset = 0.046825

        # Load calibration data
        calib_data = calib_data = {
                'acc_bias_x': 0.0,
                'acc_bias_y': 0.0,
                'acc_bias_z': 0.0,
                'ang_bias_x': 0.0,
                'ang_bias_y': 0.0,
                'ang_bias_z': 0.0,
                'ang_z2x_proj': 0.15,
                'ang_z2y_proj': -0.28
            }
        try:
            home_path = os.path.expanduser('~')
            calib_file_path = os.path.join(home_path, 'Desktop/imu_calib_data.yaml')
            calib_file = open(calib_file_path, 'r')
            calib_data = yaml.load(calib_file, Loader=yaml.FullLoader)
            print("imu_calib.yaml loaded")
            calib_file.close()
        except:
            print("imu_calib.yaml not found, using defualt values")
            
        self.acc_bias_x = calib_data['acc_bias_x']
        self.acc_bias_y = calib_data['acc_bias_y']
        self.acc_bias_z = calib_data['acc_bias_z']
        self.ang_bias_x = calib_data['ang_bias_x']
        self.ang_bias_y = calib_data['ang_bias_y']
        self.ang_bias_z = calib_data['ang_bias_z']
        self.ang_z2x_proj = calib_data['ang_z2x_proj']
        self.ang_z2y_proj = calib_data['ang_z2y_proj']
                
        self.body2cloud_trans = TransformStamped()
        self.body2cloud_trans.header.stamp = self.get_clock().now().to_msg()
        self.body2cloud_trans.header.frame_id = "body"
        self.body2cloud_trans.child_frame_id = "utlidar_lidar_1"
        self.body2cloud_trans.transform.translation.x = 0.0
        self.body2cloud_trans.transform.translation.y = 0.0
        self.body2cloud_trans.transform.translation.z = 0.0
        quat = tf_transformations.quaternion_from_euler(0, 2.87820258505555555556, 0)
        self.body2cloud_trans.transform.rotation.x = quat[0]
        self.body2cloud_trans.transform.rotation.y = quat[1]
        self.body2cloud_trans.transform.rotation.z = quat[2]
        self.body2cloud_trans.transform.rotation.w = quat[3]
        
        self.body2imu_trans = TransformStamped()
        self.body2imu_trans.header.stamp = self.get_clock().now().to_msg()
        self.body2imu_trans.header.frame_id = "body"
        self.body2imu_trans.child_frame_id = "utlidar_imu_1"
        self.body2imu_trans.transform.translation.x = 0.0
        self.body2imu_trans.transform.translation.y = 0.0
        self.body2imu_trans.transform.translation.z = 0.0
        quat = tf_transformations.quaternion_from_euler(0, 2.87820258505555555556, 3.14159265358)
        self.body2imu_trans.transform.rotation.x = quat[0]
        self.body2imu_trans.transform.rotation.y = quat[1]
        self.body2imu_trans.transform.rotation.z = quat[2]
        self.body2imu_trans.transform.rotation.w = quat[3]
        
        self.x_filter_min = -0.7
        self.x_filter_max = -0.1
        self.y_filter_min = -0.3
        self.y_filter_max = 0.3
        self.z_filter_min = -0.6 - self.cam_offset
        self.z_filter_max = 0 - self.cam_offset

    def publish_mapping_status(self):
        if self.time_mode == 'legacy':
            report = dict(ready=self.time_stamp_offset_set, fault='',
                          offset_ns=self.time_stamp_offset if self.time_stamp_offset_set else None,
                          offset_sec=self.time_stamp_offset/1e9 if self.time_stamp_offset_set else None,
                          absolute_clock_synchronization=False,
                          limitation='Historical first-cloud receipt offset includes scan/transport delay; '
                                     'no ImuTimeMapping READY/FAULT/source guard is used')
        else:
            report = self.mapping.status()
        report['mode'] = self.time_mode
        report['ros_stamp_ns'] = self.get_clock().now().nanoseconds
        self.mapping_status_pub.publish(String(data=json.dumps(report)))
                
    def is_in_filter_box(self, point):
        # Check if the point is in the filter box
        is_in_box = point[0] > self.x_filter_min and \
                    point[0] < self.x_filter_max and \
                    point[1] > self.y_filter_min and \
                    point[1] < self.y_filter_max and \
                    point[2] > self.z_filter_min and \
                    point[2] < self.z_filter_max
        return is_in_box

    def cloud_callback(self, data):
        receive_ns = self.get_clock().now().nanoseconds
        if self.time_mode == 'legacy':
            if not self.time_stamp_offset_set:
                self.time_stamp_offset = receive_ns-Time.from_msg(data.header.stamp).nanoseconds
                self.time_stamp_offset_set = True
                self.get_logger().warning(
                    f'TIME_MODE=legacy first_cloud_offset_ns={self.time_stamp_offset}; fixed for this run')
        elif not self.mapping.ready:
            return
                
        cloud_arr = pc2.read_points_list(data)
        if not cloud_arr:
            return
        points = np.array(cloud_arr)

        transform = self.body2cloud_trans.transform
        mat = quat2mat(np.array([transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z]))
        translation = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
        
        transformed_points = points
        transformed_points[:, 0:3] = points[:, 0:3] @ mat.T + translation
        transformed_points[:, 2] -= self.cam_offset
        i = 0
        remove_list = []
        transformed_points = transformed_points.tolist()
        for i in range(len(transformed_points)):
            transformed_points[i][4] = int(transformed_points[i][4])
            if self.is_in_filter_box(transformed_points[i]):
                remove_list.append(i)

        remove_list.sort(reverse=True)

        for id_to_remove in remove_list:
            del transformed_points[id_to_remove]

        if self.time_mode == 'legacy':
            mapped_stamp = Time.from_msg(data.header.stamp).nanoseconds+self.time_stamp_offset
        else:
            time_index = next((i for i, f in enumerate(data.fields) if f.name == 'time'), None)
            if time_index is None or not transformed_points:
                return
            relative = [p[time_index] for p in transformed_points]
            if not all(np.isfinite(v) and v >= 0 for v in relative):
                self.mapping.fault = 'invalid L1 point relative time'
                return
            mapped_stamp = self.mapping.map_stamp(
                'cloud', Time.from_msg(data.header.stamp).nanoseconds,
                receive_ns, round(max(relative)*1e9))
        if mapped_stamp is None:
            return
        
        elevated_cloud = pc2.create_cloud(deepcopy(data.header), data.fields, transformed_points)
        elevated_cloud.header.stamp = Time(nanoseconds=mapped_stamp).to_msg()
        elevated_cloud.header.frame_id = "body"
        elevated_cloud.is_dense = data.is_dense

        self.cloud_pub.publish(elevated_cloud)
            
    def transform_vector(self, vector, rotation):
        # Transform a vector using a given quaternion rotation
        q_vector = [vector.x, vector.y, vector.z, 0.0]
        q_rotated = tf_transformations.quaternion_multiply(
            tf_transformations.quaternion_multiply(rotation, q_vector),
            tf_transformations.quaternion_conjugate(rotation)
        )
        
        ret_vec = Vector3()
        ret_vec.x = q_rotated[0]
        ret_vec.y = q_rotated[1]
        ret_vec.z = q_rotated[2]
        return ret_vec


    def imu_callback(self, data, info):
        receive_ns = self.get_clock().now().nanoseconds
        stamp_ns = Time.from_msg(data.header.stamp).nanoseconds
        if self.time_mode == 'legacy':
            # Share the first cloud's offset; do not emit a second epoch at startup.
            if not self.time_stamp_offset_set:
                return
            mapped_stamp = stamp_ns+self.time_stamp_offset
        else:
            if not self.mapping.observe_imu(stamp_ns, info.get('source_timestamp', 0),
                                            receive_ns, time.monotonic_ns()):
                return
            mapped_stamp = self.mapping.map_stamp('imu', stamp_ns, receive_ns)
        if mapped_stamp is None:
            return
        trans = np.zeros(3)
        trans[0] = self.body2imu_trans.transform.translation.x
        trans[1] = self.body2imu_trans.transform.translation.y
        trans[2] = self.body2imu_trans.transform.translation.z
        
        rot = np.zeros(4)
        rot[0] = self.body2imu_trans.transform.rotation.x
        rot[1] = self.body2imu_trans.transform.rotation.y
        rot[2] = self.body2imu_trans.transform.rotation.z
        rot[3] = self.body2imu_trans.transform.rotation.w
        
        transformed_orientation = tf_transformations.quaternion_multiply(rot, [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        
        x = data.angular_velocity.x
        y = -data.angular_velocity.y
        z = -data.angular_velocity.z
        
        theta = 15.1 / 180 * 3.1415926

        x2 = np.cos(theta) * x - np.sin(theta) * z
        y2 = y
        z2 = np.sin(theta) * x + np.cos(theta) * z

        x2 -= self.ang_bias_x
        y2 -= self.ang_bias_y
        z2 -= self.ang_bias_z
        
        x_comp_rate = self.ang_z2x_proj
        y_comp_rate = self.ang_z2y_proj
        
        x2 += x_comp_rate * z2
        y2 += y_comp_rate * z2
        
        transformed_angular_velocity = Vector3()
        transformed_angular_velocity.x = x2
        transformed_angular_velocity.y = y2
        transformed_angular_velocity.z = z2
        
        acc_x = data.linear_acceleration.x
        acc_y = -data.linear_acceleration.y
        acc_z = -data.linear_acceleration.z
        
        acc_x2 = np.cos(theta) * acc_x - np.sin(theta) * acc_z
        acc_y2 = acc_y
        acc_z2 = np.sin(theta) * acc_x + np.cos(theta) * acc_z
        transformed_linear_acceleration = Vector3()
        transformed_linear_acceleration.x = acc_x2 - self.acc_bias_x
        transformed_linear_acceleration.y = acc_y2 - self.acc_bias_y
        transformed_linear_acceleration.z = acc_z2 - self.acc_bias_z
        

        transformed_imu = Imu()
        transformed_imu.header.stamp = data.header.stamp
        transformed_imu.header.frame_id = 'body'
        transformed_imu.orientation.x = transformed_orientation[0]
        transformed_imu.orientation.y = transformed_orientation[1]
        transformed_imu.orientation.z = transformed_orientation[2]
        transformed_imu.orientation.w = transformed_orientation[3]
        transformed_imu.angular_velocity = transformed_angular_velocity
        transformed_imu.linear_acceleration = transformed_linear_acceleration
        
        transformed_imu.header.stamp = Time(nanoseconds=mapped_stamp).to_msg()
        
        self.imu_raw_pub.publish(transformed_imu)
        
        transformed_imu.orientation.x = 0.0
        transformed_imu.orientation.y = 0.0
        transformed_imu.orientation.z = 0.0
        transformed_imu.orientation.w = 1.0
        
        transformed_imu.linear_acceleration.x = 0.0
        transformed_imu.linear_acceleration.y = 0.0
        transformed_imu.linear_acceleration.z = 0.0
        
        self.imu_pub.publish(transformed_imu)

def main(args=None):
    rclpy.init(args=args)

    transform_node = Repuber()

    executor = TimestampExecutor()
    executor.add_node(transform_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        transform_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
