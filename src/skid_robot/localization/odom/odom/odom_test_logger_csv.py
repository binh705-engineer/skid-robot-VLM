#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def cov6_to_p3(cov6):
    # Extract [x, y, yaw] covariance from ROS 6x6 pose covariance
    return np.array([
        [cov6[0],  cov6[1],  cov6[5]],
        [cov6[6],  cov6[7],  cov6[11]],
        [cov6[30], cov6[31], cov6[35]],
    ], dtype=float)


class OdomTestLoggerCsv(Node):
    def __init__(self):
        super().__init__('odom_test_logger_csv')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('csv_path', '/tmp/odom_test_log.csv')

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.csv_path = str(self.get_parameter('csv_path').value)

        self.last_odom = None

        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.cb,
            10
        )

        self.get_logger().info(f"Listening to {self.odom_topic}")
        self.get_logger().info(f"CSV output: {self.csv_path}")
        self.get_logger().info("Run the test, then press Ctrl-C to print summary, ask for ground truth, and save CSV.")

    def cb(self, msg: Odometry):
        self.last_odom = msg

    def save_row(self, row: dict):
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        file_exists = os.path.exists(self.csv_path)

        fieldnames = [
            "timestamp",
            "x_odom", "y_odom", "yaw_odom_rad",
            "P_xx", "P_xy", "P_xyaw",
            "P_yx", "P_yy", "P_yyaw",
            "P_yawx", "P_yawy", "P_yawyaw",
            "x_true", "y_true", "yaw_true_rad",
            "err_x", "err_y", "err_yaw_rad",
        ]

        with open(self.csv_path, mode="a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if not file_exists:
                writer.writeheader()
            writer.writerow(row)

    def report_and_save(self):
        if self.last_odom is None:
            print("No odom message received.")
            return

        odom = self.last_odom

        x_odom = float(odom.pose.pose.position.x)
        y_odom = float(odom.pose.pose.position.y)
        yaw_odom = float(quat_to_yaw(odom.pose.pose.orientation))
        P_end = cov6_to_p3(odom.pose.covariance)

        print("\n========== ODOM TEST SUMMARY ==========")
        print(f"Pose odom cuối:")
        print(f"  x   = {x_odom:.6f} m")
        print(f"  y   = {y_odom:.6f} m")
        print(f"  yaw = {yaw_odom:.6f} rad")

        print("\nP_end (3x3 from pose.covariance):")
        print(P_end)

        try:
            x_true = float(input("\nNhập x_true [m]   = ").strip())
            y_true = float(input("Nhập y_true [m]   = ").strip())
            yaw_true = float(input("Nhập yaw_true [rad] = ").strip())
        except Exception as e:
            print(f"Input error: {e}")
            return

        ex = x_true - x_odom
        ey = y_true - y_odom
        eyaw = wrap_angle(yaw_true - yaw_odom)

        print("\nSai số thật:")
        print(f"  e_x   = {ex:.6f} m")
        print(f"  e_y   = {ey:.6f} m")
        print(f"  e_yaw = {eyaw:.6f} rad")

        row = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "x_odom": x_odom,
            "y_odom": y_odom,
            "yaw_odom_rad": yaw_odom,
            "P_xx": float(P_end[0, 0]),
            "P_xy": float(P_end[0, 1]),
            "P_xyaw": float(P_end[0, 2]),
            "P_yx": float(P_end[1, 0]),
            "P_yy": float(P_end[1, 1]),
            "P_yyaw": float(P_end[1, 2]),
            "P_yawx": float(P_end[2, 0]),
            "P_yawy": float(P_end[2, 1]),
            "P_yawyaw": float(P_end[2, 2]),
            "x_true": x_true,
            "y_true": y_true,
            "yaw_true_rad": yaw_true,
            "err_x": ex,
            "err_y": ey,
            "err_yaw_rad": eyaw,
        }

        self.save_row(row)
        print(f"\nĐã ghi 1 dòng vào CSV: {self.csv_path}")
        print("======================================\n")


def main():
    rclpy.init()
    node = OdomTestLoggerCsv()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report_and_save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
