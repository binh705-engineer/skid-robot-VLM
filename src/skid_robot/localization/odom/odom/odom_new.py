#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_from_euler


class EncoderIMUToOdomNode(Node):
    def __init__(self):
        super().__init__('odom2')

        # Parameters
        self.declare_parameter('wheel_radius', 0.098)
        self.declare_parameter('wheel_base', 0.48)
        self.declare_parameter('encoder_feedback_topic', 'zlac_encoder')
        self.declare_parameter('odom_topic', 'odom')

        # Odometry error model coefficients
        # Qu = diag(kr * |ds_r|, kl * |ds_l|)
        # Tune these experimentally for your robot / floor / encoder quality.
        self.declare_parameter('kr', 0.02)
        self.declare_parameter('kl', 0.02)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.encoder_feedback_topic = str(self.get_parameter('encoder_feedback_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.kr = float(self.get_parameter('kr').value)
        self.kl = float(self.get_parameter('kl').value)

        # TF wheel joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0

        # State estimate: x, y, theta
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Covariance of pose estimate [x, y, theta]
        # Small initial uncertainty; tune if you want to reflect initialization quality.
        self.P = np.diag([1e-6, 1e-6, 1e-6]).astype(float)

        self.last_time = None
        self.left_rpm = None
        self.right_rpm = None

        # Timer
        self.timer = self.create_timer(0.02, self.update_odom)

        # Subscriber: [fb_rpm_left, fb_rpm_right]
        self.encoder_sub = self.create_subscription(
            Int32MultiArray,
            self.encoder_feedback_topic,
            self.encoder_callback,
            10
        )

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        self.get_logger().info(
            f"EncoderIMUToOdomNode started. Subscribing to: {self.encoder_feedback_topic}"
        )
        self.get_logger().info(
            f"Using odom covariance model with kr={self.kr}, kl={self.kl}"
        )

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def build_pose_covariance_6x6(P3: np.ndarray) -> list:
        """
        Embed 3x3 covariance [x, y, yaw] into ROS 6x6 pose covariance.
        Index mapping (row-major):
            x   -> 0
            y   -> 7
            yaw -> 35
        Cross terms:
            x-y    -> 1, 6
            x-yaw  -> 5, 30
            y-yaw  -> 11, 31
        Other axes (z, roll, pitch) set large to indicate unused.
        """
        cov = [0.0] * 36

        # Large uncertainty for unused axes in a ground robot
        cov[14] = 1e6  # z-z
        cov[21] = 1e6  # roll-roll
        cov[28] = 1e6  # pitch-pitch

        # Useful 3x3 block
        cov[0] = float(P3[0, 0])
        cov[1] = float(P3[0, 1])
        cov[5] = float(P3[0, 2])

        cov[6] = float(P3[1, 0])
        cov[7] = float(P3[1, 1])
        cov[11] = float(P3[1, 2])

        cov[30] = float(P3[2, 0])
        cov[31] = float(P3[2, 1])
        cov[35] = float(P3[2, 2])

        return cov

    @staticmethod
    def build_twist_covariance_6x6(var_v: float, var_w: float) -> list:
        """
        Embed twist covariance for linear.x and angular.z.
        Other axes are set large / unused for a differential-drive ground robot.
        """
        cov = [0.0] * 36

        cov[0] = float(var_v)   # linear x
        cov[14] = 1e6           # linear z
        cov[21] = 1e6           # angular x
        cov[28] = 1e6           # angular y
        cov[35] = float(var_w)  # angular z

        # Unused velocity directions
        cov[7] = 1e6
        cov[7] = 1e6

        return cov

    def encoder_callback(self, msg: Int32MultiArray):
        # ESP32 sends 2 elements: [fb_rpm_left, fb_rpm_right]
        if len(msg.data) < 2:
            self.get_logger().warn(
                f"{self.encoder_feedback_topic} thiếu phần tử: nhận {len(msg.data)}, cần 2"
            )
            return

        self.left_rpm = float(msg.data[0])
        self.right_rpm = -float(msg.data[1])  # invert right wheel sign

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

        # RPM -> wheel linear velocity (m/s)
        left_velocity = (self.left_rpm * 2.0 * math.pi * self.wheel_radius) / 60.0
        right_velocity = (self.right_rpm * 2.0 * math.pi * self.wheel_radius) / 60.0

        # Wheel angular speed for JointState
        left_rad_s = self.left_rpm * 2.0 * math.pi / 60.0
        right_rad_s = self.right_rpm * 2.0 * math.pi / 60.0

        # Incremental wheel distances in this cycle
        ds_r = right_velocity * dt
        ds_l = left_velocity * dt

        # Differential-drive motion model
        ds = (ds_r + ds_l) / 2.0
        dtheta = (ds_r - ds_l) / self.wheel_base
        theta_mid = self.theta + dtheta / 2.0

        if any(
            math.isnan(v) or math.isinf(v)
            for v in [left_velocity, right_velocity, ds_r, ds_l, ds, dtheta, theta_mid]
        ):
            return

        # State propagation
        x_new = self.x + ds * math.cos(theta_mid)
        y_new = self.y + ds * math.sin(theta_mid)
        theta_new = self.normalize_angle(self.theta + dtheta)

        # Jacobian wrt previous pose p = [x, y, theta]
        Fp = np.array([
            [1.0, 0.0, -ds * math.sin(theta_mid)],
            [0.0, 1.0,  ds * math.cos(theta_mid)],
            [0.0, 0.0,  1.0]
        ], dtype=float)

        # Jacobian wrt wheel increments u = [ds_r, ds_l]
        # x' = x + ds*cos(theta_mid)
        # y' = y + ds*sin(theta_mid)
        # theta' = theta + (ds_r - ds_l)/b
        Fu = np.array([
            [
                0.5 * math.cos(theta_mid) - (ds * math.sin(theta_mid)) / (2.0 * self.wheel_base),
                0.5 * math.cos(theta_mid) + (ds * math.sin(theta_mid)) / (2.0 * self.wheel_base),
            ],
            [
                0.5 * math.sin(theta_mid) + (ds * math.cos(theta_mid)) / (2.0 * self.wheel_base),
                0.5 * math.sin(theta_mid) - (ds * math.cos(theta_mid)) / (2.0 * self.wheel_base),
            ],
            [
                1.0 / self.wheel_base,
                -1.0 / self.wheel_base,
            ]
        ], dtype=float)

        # Wheel-distance noise covariance.
        # Per the book idea: variance grows with traveled distance.
        Qu = np.diag([
            self.kr * abs(ds_r),
            self.kl * abs(ds_l)
        ]).astype(float)

        # Covariance propagation
        self.P = Fp @ self.P @ Fp.T + Fu @ Qu @ Fu.T

        # Keep symmetry numerically
        self.P = 0.5 * (self.P + self.P.T)

        # Update state
        self.x = x_new
        self.y = y_new
        self.theta = theta_new

        # Quaternion
        q = quaternion_from_euler(0.0, 0.0, self.theta)

        # Twist covariance from same wheel noise model
        # v = ds/dt, w = dtheta/dt
        J_twist = np.array([
            [1.0 / (2.0 * dt), 1.0 / (2.0 * dt)],
            [1.0 / (self.wheel_base * dt), -1.0 / (self.wheel_base * dt)]
        ], dtype=float)

        twist_cov_2x2 = J_twist @ Qu @ J_twist.T
        var_v = float(twist_cov_2x2[0, 0])
        var_w = float(twist_cov_2x2[1, 1])

        # Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.orientation = Quaternion(
            x=float(q[0]),
            y=float(q[1]),
            z=float(q[2]),
            w=float(q[3])
        )

        odom.twist.twist.linear.x = float((left_velocity + right_velocity) / 2.0)
        odom.twist.twist.angular.z = float((right_velocity - left_velocity) / self.wheel_base)

        odom.pose.covariance = self.build_pose_covariance_6x6(self.P)
        odom.twist.covariance = self.build_twist_covariance_6x6(var_v, var_w)

        self.odom_pub.publish(odom)

        # Joint states for wheel TF
        self.left_wheel_angle += left_rad_s * dt
        self.right_wheel_angle += right_rad_s * dt

        js = JointState()
        js.header.stamp = current_time.to_msg()
        js.name = [
            'wheel_front_left_jt',
            'wheel_back_left_jt',
            'wheel_front_right_jt',
            'wheel_back_right_jt'
        ]
        js.position = [
            self.left_wheel_angle,
            self.left_wheel_angle,
            self.right_wheel_angle,
            self.right_wheel_angle
        ]
        self.joint_pub.publish(js)


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
