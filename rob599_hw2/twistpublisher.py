#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class TwistPublisher(Node):

    def __init__(self):
        super().__init__('twist_publisher')
        self.publisher = self.create_publisher(Twist, 'speed_in', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_twist)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def publish_twist(self):
        msg = Twist()
        self.linear_velocity += 0.5
        self.angular_velocity += 0.5
        msg.linear.x = self.linear_velocity  # m/s
        msg.angular.z = self.angular_velocity  # rad/s
        self.publisher.publish(msg)
        self.get_logger().info(f"Publishing Twist message: {msg.linear.x} m/s, {msg.angular.z} rad/s")


def main(args=None):
    rclpy.init(args=args)
    twist_publisher = TwistPublisher()
    rclpy.spin(twist_publisher)
    twist_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
