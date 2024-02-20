#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
# It existed, third rule of ROS2 Club
from std_srvs.srv import SetBool

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

        # Watchdog parameters
        self.declare_parameter('with_watchdog', True)
        self.declare_parameter('watchdog_period', 7.0)
        self.with_watchdog = self.get_parameter('with_watchdog').get_parameter_value().bool_value
        self.watchdog_period = self.get_parameter('watchdog_period').get_parameter_value().double_value
        self.timer = None

        self.linear_max = self.get_parameter('linear_max').get_parameter_value().double_value
        self.angular_max = self.get_parameter('angular_max').get_parameter_value().double_value

        if self.with_watchdog:
            self.start_watchdog()

        # Create a service to apply brakes
        self.apply_brakes_service = self.create_service(SetBool, 'apply_brakes', self.apply_brakes_callback)
        self.brake_timer = self.create_timer(0.1, self.publish_brakes_callback)
        self.publish_brakes = False


    def start_watchdog(self):
        if self.timer is not None:
            self.timer.cancel()
        self.timer = self.create_timer(self.watchdog_period, self.watchdog_callback)
        self.get_logger().info("Watchdog timer started")

    def watchdog_callback(self):
            stop_msg = Twist()

            stop_msg.linear.x = 0.0
            stop_msg.linear.y = 0.0
            stop_msg.linear.z = 0.0
            stop_msg.angular.x = 0.0
            stop_msg.angular.y = 0.0
            stop_msg.angular.z = 0.0

            self.get_logger().info(f"Zero Twist Message Sent: {stop_msg.linear.x} m/s, {stop_msg.angular.z} rad/s")

        #self.publisher.publish(stop_msg)


    def speed_callback(self, msg):

        if self.with_watchdog:
            self.start_watchdog()
            
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

    def apply_brakes_callback(self, request, response):
        if request.data:
            # Apply brakes
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.linear.y = 0.0
            stop_msg.linear.z = 0.0
            stop_msg.angular.x = 0.0
            stop_msg.angular.y = 0.0
            stop_msg.angular.z = 0.0
            self.get_logger().info(f"Brakes applied. Speed set to :{stop_msg.linear.x} m/s, {stop_msg.angular.z} rad/s")

            self.publish_brakes = True
        else:
            # Revert to original behavior
            self.publish_brakes = False
            pass  # Nothing to do
        response.success = True
        response.message = 'Brakes applied' if request.data else 'Brakes released'
        return response
    
    def publish_brakes_callback(self):
        # Publish the apply_brakes_callback periodically at 10 Hz
        if self.publish_brakes:
            request = SetBool.Request()
            request.data = True  # Always set to True to keep brakes applied
            self.apply_brakes_callback(request, SetBool.Response())
            self.get_logger().info("Pumping the brakes at 10 Hz")
        else:
            # Release the brakes
            request = SetBool.Request()
            request.data = False
            self.apply_brakes_callback(request, SetBool.Response())


def main(args=None):
    rclpy.init(args=args)
    speed_limiter = SpeedLimiter()
    rclpy.spin(speed_limiter)
    speed_limiter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
