#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter

class SpeedLimiter(Node):

    def __init__(self):
        super().__init__('speed_limiter')
        self.subscription = self.create_subscription(
            Twist,
            'speed_in',
            self.speed_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'speed_out', 10)
        # Default velocity constraints
        self.declare_parameter('linear_max', 21.0)
        self.declare_parameter('angular_max', 21.0)
        # Get the parameter values
        self.linear_max = self.get_parameter('linear_max').get_parameter_value().double_value
        self.angular_max = self.get_parameter('angular_max').get_parameter_value().double_value


    def speed_callback(self, msg):
        # For Debugging 
        self.get_logger().info(f"Received Twist message: {msg.linear.x} m/s, {msg.angular.z} rad/s")
        # Limit linear speed
        if abs(msg.linear.x) > self.linear_max:
            if msg.linear.x > 0:
                msg.linear.x = self.linear_max  
            else:
                msg.linear.x = -self.linear_max

        # Limit angular speed
        if abs(msg.angular.z) > self.angular_max:
            if msg.angular.z > 0:
                msg.angular.z = self.angular_max  
            else:
                msg.angular.z = -self.angular_max

        # For Debugging 
        self.get_logger().info(f"Modified Twist message: {msg.linear.x} m/s, {msg.angular.z} rad/s")

        # Publish the limited speed
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    speed_limiter = SpeedLimiter()
    rclpy.spin(speed_limiter)
    speed_limiter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
