#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Differential Drive Kinematics Node for ROS 2.

This module converts high-level twist commands (linear and angular velocities)
into low-level wheel velocities (RPM) for a differential drive mobile robot,
handling hardware-specific coordinate system conversions.

Author: Pham Duc Duy - https://github.com/phamduyaaaa

Version:
    1.0.0
"""

import math
from typing import List, Optional

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class KinematicsNode(Node):
    """ROS 2 Node that calculates individual wheel RPM from geometry_msgs/Twist.

    This class handles the inverse kinematics for a two-wheeled differential
    drive robot, including scaling from linear/angular velocities to RPM and
    accounting for hardware-level phase inversions.

    Attributes:
        wheel_radius (float): The radius of the robot's wheels in meters.
        wheel_separation (float): The distance between the track centers of the
            two drive wheels in meters.
        subscription (rclpy.subscription.Subscription): Subscriber for the
            '/cmd_vel' topic.
        rpm_publisher (rclpy.publisher.Publisher): Publisher for the
            '/wheel_rpm' topic.
    """

    def __init__(self) -> None:
        """Initializes the KinematicsNode with parameters, publishers, and subscribers."""
        super().__init__('differential_kinematics_node')
        
        # ---------------------------------------------------------
        # PHYSICAL ROBOT PARAMETERS (Configure per robot spec)
        # ---------------------------------------------------------
        self.wheel_radius: float = 0.1      # Wheel radius (meters)
        self.wheel_separation: float = 0.47    # Track width / wheelbase (meters)
        
        # ---------------------------------------------------------
        # ROS 2 INTERFACES INITIALIZATION
        # ---------------------------------------------------------
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.rpm_publisher = self.create_publisher(
            Int32MultiArray,
            '/wheel_rpm',
            10
        )
        
        self.get_logger().info(
            "Kinematics Node successfully initialized. "
        )

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Processes incoming Twist messages and publishes calculated wheel RPMs.

        Performs inverse kinematics calculation: (v, w) -> (v_left, v_right),
        converts linear velocity to rotational speed (RPM), applies hardware
        inversion logic for the right motor, and publishes the result.

        Args:
            msg (Twist): The target linear and angular velocity command from the
                navigation stack.
        """
        # Extract linear velocity along the x-axis and angular velocity around the z-axis
        linear_v: float = msg.linear.x
        angular_w: float = msg.angular.z

        # 1. Compute theoretical linear velocity for each wheel (m/s)
        # Using standard differential drive kinematics equations
        v_left: float = linear_v - (angular_w * self.wheel_separation / 2.0)
        v_right: float = linear_v + (angular_w * self.wheel_separation / 2.0)

        # 2. Convert linear velocity (m/s) to Revolutions Per Minute (RPM)
        wheel_circumference: float = 2.0 * math.pi * self.wheel_radius
        
        # Avoid DivisionByZero if wheel_circumference is misconfigured
        if wheel_circumference <= 0.0:
            self.get_logger().error("Invalid wheel radius configured. Calculation aborted.")
            return

        rpm_left: float = (v_left * 60.0) / wheel_circumference
        rpm_right: float = (v_right * 60.0) / wheel_circumference

        # ---------------------------------------------------------
        # 3. HARDWARE FIRMWARE COORDINATE ALIGNMENT
        # ---------------------------------------------------------
        # As specified in the firmware logic (e.g., compute_rc_rpm_right in new_2.ino),
        # the right motor requires a negative RPM value to rotate in the forward direction
        # due to its symmetrical mounting orientation on the chassis frame.
        rpm_right_aligned: float = rpm_right

        # Pack data into standard Int32MultiArray payload safely casting to integer
        rpm_msg: Int32MultiArray = Int32MultiArray()
        rpm_msg.data = [int(rpm_left), int(rpm_right_aligned)]

        # Publish the target RPM array to the actuators
        self.rpm_publisher.publish(rpm_msg)
        
        # Debug logger with explicit formatting
        self.get_logger().debug(
            f"Cmd Received -> V: {linear_v:.2f} m/s, W: {angular_w:.2f} rad/s | "
            f"Output RPM -> Left: {int(rpm_left)}, Right: {int(rpm_right_aligned)}"
        )


def main(args: Optional[List[str]] = None) -> None:
    """Main entry point for the ROS 2 node execution loop.

    Handles initialization, execution spin lock, and graceful shutdown contexts.

    Args:
        args (list of str, optional): Command line arguments passed to the node.
    """
    rclpy.init(args=args)
    node: KinematicsNode = KinematicsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down gracefully...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
