#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
import math


class EncoderIMUToOdomNode(Node):
    def __init__(self):
        super().__init__('odom2')

        # Parameters (đổi tên theo driver mới)
        self.declare_parameter('wheel_radius', 0.098)
        self.declare_parameter('wheel_base', 0.48)
        self.declare_parameter('encoder_feedback_topic', 'zlac_encoder')
        self.declare_parameter('odom_topic', 'odom')

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.encoder_feedback_topic = self.get_parameter('encoder_feedback_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value

        # TF wheel
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = None
        self.left_rpm = None
        self.right_rpm = None

        # Timer
        self.timer = self.create_timer(0.02, self.update_odom)

        # Subscriber: 1 topic duy nhất từ ESP32 (driver mới)
        # Int32MultiArray: [fb_rpm_left, fb_rpm_right] — chỉ 2 phần tử
        self.encoder_sub = self.create_subscription(
            Int32MultiArray,
            self.encoder_feedback_topic,
            self.encoder_callback,
            10
        )

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        # TF
        #self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info(
            f"EncoderIMUToOdomNode started. Subscribing to: {self.encoder_feedback_topic}"
        )

    def encoder_callback(self, msg: Int32MultiArray):
        # ESP32 gửi 2 phần tử: [fb_rpm_left, fb_rpm_right]
        if len(msg.data) < 2:
            self.get_logger().warn(
                f"{self.encoder_feedback_topic} thiếu phần tử: nhận {len(msg.data)}, cần 2"
            )
            return

        self.left_rpm = float(msg.data[0])
        self.right_rpm = -float(msg.data[1])  # đảo dấu bánh phải

    def update_odom(self):
        if self.left_rpm is None or self.right_rpm is None:
            return

        current_time = self.get_clock().now()

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        if dt <= 0.0:
            return

        # RPM → m/s
        left_velocity = (self.left_rpm * 2 * math.pi * self.wheel_radius) / 60.0
        right_velocity = (self.right_rpm * 2 * math.pi * self.wheel_radius) / 60.0

        # TF wheel
        left_rad_s = self.left_rpm * 2.0 * math.pi / 60.0
        right_rad_s = self.right_rpm * 2.0 * math.pi / 60.0
        self.left_wheel_angle += left_rad_s * dt
        self.right_wheel_angle += right_rad_s * dt

        linear_velocity = (left_velocity + right_velocity) / 2.0
        angular_velocity = (right_velocity - left_velocity) / self.wheel_base

        if any(math.isnan(v) or math.isinf(v)
               for v in [left_velocity, right_velocity, linear_velocity, angular_velocity]):
            return

        # Integrate
        self.x += linear_velocity * math.cos(self.theta) * dt
        self.y += linear_velocity * math.sin(self.theta) * dt
        self.theta += angular_velocity * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        q = quaternion_from_euler(0, 0, self.theta)

        # Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3]
        )

        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        odom.pose.covariance = [
            0.01,    0.0,    0.0,   0.0,    0.0,    0.0,
            0.0,     0.01,   0.0,   0.0,    0.0,    0.0,
            0.0,     0.0,    1e6,   0.0,    0.0,    0.0,
            0.0,     0.0,    0.0,   1e6,    0.0,    0.0,
            0.0,     0.0,    0.0,   0.0,    1e6,    0.0,
            0.0,     0.0,    0.0,   0.0,    0.0,    0.5
        ]

        odom.twist.covariance = [
            0.001,    0.0,    0.0,   0.0,    0.0,    0.0,
            0.0,     1e6,    0.0,   0.0,    0.0,    0.0,
            0.0,     0.0,    1e6,   0.0,    0.0,    0.0,
            0.0,     0.0,    0.0,   1e6,    0.0,    0.0,
            0.0,     0.0,    0.0,   0.0,    1e6,    0.0,
            0.0,     0.0,    0.0,   0.0,    0.0,    0.01
        ]

        self.odom_pub.publish(odom)

        # TF wheel
        js = JointState()
        js.header.stamp = current_time.to_msg()
        js.name = ['wheel_front_left_jt', 'wheel_back_left_jt', 'wheel_front_right_jt', 'wheel_back_right_jt']
        js.position = [self.left_wheel_angle, self.left_wheel_angle, self.right_wheel_angle, self.right_wheel_angle]

        self.joint_pub.publish(js)

        # TF
        #tf = TransformStamped()
        #tf.header.stamp = odom.header.stamp
        #tf.header.frame_id = 'odom'
        #tf.child_frame_id = 'base_footprint'
        #tf.transform.translation.x = self.x
        #tf.transform.translation.y = self.y
        #tf.transform.rotation = odom.pose.pose.orientation

        #self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderIMUToOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
