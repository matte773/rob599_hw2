#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter


class TwistChecker(Node):

    def __init__(self):
        super().__init__('twist_checker')
        self.subscription = self.create_subscription(
            Twist,
            'speed_in',
            self.twist_callback,
            10)

        self.declare_parameter('linear_max', 21.0)
        self.declare_parameter('angular_max', 21.0)

        self.get_parameters()

        self.message_count = 0
        self.outside_count = 0

        self.timer = self.create_timer(30, self.log_statistics)

    def get_parameters(self):
        self.linear_max = self.get_parameter('linear_max').get_parameter_value().double_value
        self.angular_max = self.get_parameter('angular_max').get_parameter_value().double_value

    def twist_callback(self, msg):
        self.message_count += 1
        # Check if linear velocity is within limits
        if abs(msg.linear.x) > self.linear_max or abs(msg.linear.y) > self.linear_max or abs(
                msg.linear.z) > self.linear_max:
            self.outside_count += 1
        # Check if angular velocity is within limits
        if abs(msg.angular.x) > self.angular_max or abs(msg.angular.y) > self.angular_max or abs(
                msg.angular.z) > self.angular_max:
            self.outside_count += 1

    def log_statistics(self):
        proportion = self.outside_count / self.message_count if self.message_count != 0 else 0
        percent = (proportion/2) * 100
        self.get_logger().info(f"Received {self.message_count} messages in the last 30 seconds. "
                               f"Percentage of messages outside the speed bounds: {percent}%")
        # Reset statistics for the next 30-second interval
        self.message_count = 0
        self.outside_count = 0


def main(args=None):
    rclpy.init(args=args)
    twist_checker = TwistChecker()
    rclpy.spin(twist_checker)
    twist_checker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
