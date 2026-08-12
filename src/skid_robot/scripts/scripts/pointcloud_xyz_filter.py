#!/usr/bin/env python3
"""
Chuan hoa PointCloud2 tu Velodyne (co field intensity, ring, time)
thanh PointCloud2 chi gom x, y, z (FLOAT32), point_step = 16 byte
(co 4 byte padding de align GPU), phu hop voi NitrosPointCloud cua
Isaac ROS (pointcloud_to_flatscan).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct


class PointCloudXYZFilter(Node):
    def __init__(self):
        super().__init__('pointcloud_xyz_filter')

        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/velodyne_points/xyz')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # Input: giu nguyen sensor QoS (BEST_EFFORT) vi day la QoS goc cua Velodyne driver
        self.sub = self.create_subscription(
            PointCloud2, input_topic, self.callback, qos_profile_sensor_data)

        # Output: dung RELIABLE vi pointcloud_to_flatscan (NITROS) mac dinh
        # subscribe voi QoS "DEFAULT" = RELIABLE, khong phai BEST_EFFORT
        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.pub = self.create_publisher(
            PointCloud2, output_topic, output_qos)

        self.get_logger().info(
            f'Subscribing: {input_topic}  ->  Publishing: {output_topic}')

    def callback(self, msg: PointCloud2):
        # Tim offset cua x, y, z trong message goc
        offsets = {f.name: f.offset for f in msg.fields}
        if not all(k in offsets for k in ('x', 'y', 'z')):
            self.get_logger().warn('Khong tim thay du field x,y,z, bo qua message nay')
            return

        n_points = msg.width * msg.height
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_points, msg.point_step)

        # Doc x,y,z (FLOAT32, little-endian) tu buffer goc theo offset
        x = raw[:, offsets['x']:offsets['x']+4].copy().view(np.float32).reshape(-1)
        y = raw[:, offsets['y']:offsets['y']+4].copy().view(np.float32).reshape(-1)
        z = raw[:, offsets['z']:offsets['z']+4].copy().view(np.float32).reshape(-1)

        # Loc bo diem NaN/Inf (Velodyne driver co the phat sinh)
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x, y, z = x[valid], y[valid], z[valid]
        n_valid = x.shape[0]

        # Tao buffer moi: point_step = 16 byte (x,y,z + 4 byte padding)
        out = np.zeros((n_valid, 4), dtype=np.float32)
        out[:, 0] = x
        out[:, 1] = y
        out[:, 2] = z
        # out[:, 3] la padding, giu = 0

        new_msg = PointCloud2()
        new_msg.header = msg.header
        new_msg.height = 1
        new_msg.width = n_valid
        new_msg.is_bigendian = False
        new_msg.point_step = 16
        new_msg.row_step = 16 * n_valid
        new_msg.is_dense = True
        new_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        new_msg.data = out.tobytes()

        self.pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudXYZFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
