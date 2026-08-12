import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_period = 0.00001
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.start_time = time.time()
        self.rotate_time = 0
        self.gmsg = Twist()
        self.state = "di_thang"

    def timer_callback(self):
        current_time = time.time()
        if self.state == "di_thang":
            if current_time - self.start_time >= 25.0:
                self.rotate_time = time.time()
                self.state = "quay"
                self.get_logger().info("Finished publishing messages.")
            else:
                self.gmsg.linear.x  = 0.2
                self.gmsg.angular.z  = 0.0
        elif self.state == "quay":
            if current_time-self.rotate_time>=4.0:
                self.gmsg.linear.x  = 0.0
                self.gmsg.angular.z  = 0.0
                self.publisher_.publish(self.gmsg)
                rclpy.shutdown()
                return
            else:
                self.gmsg.linear.x = 0.0
                self.gmsg.angular.z  = 0.0

        self.publisher_.publish(self.gmsg)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

if __name__ == '__main__':
    main()
