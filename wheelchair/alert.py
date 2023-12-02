#!/usr/bin/python3.10

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from playsound import playsound


def calculate_speed(linear, angular):
    linear_speed = (linear.x ** 2 + linear.y ** 2 + linear.z ** 2) ** 0.5
    angular_speed = (angular.x ** 2 + angular.y ** 2 + angular.z ** 2) ** 0.5
    return linear_speed + angular_speed


class Alert(Node):
    def __init__(self):
        super().__init__('speed_alert')
        self.subscription_ = self.create_subscription(
            Odometry,
            "/model/wheelchair/odometry",
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(
            Twist,
            "model/wheelchair/cmd_vel",
            10
        )
        self.MAX_SPEED = 5.46807  # equivalent to 5 km/h (ratio is 1:10 in simulation)

    def listener_callback(self, msg):
        speed = calculate_speed(msg.twist.twist.linear, msg.twist.twist.angular)

        if speed > self.MAX_SPEED:
            playsound('too_fast.mp3')
            self.get_logger().info('Speed is too high! Speed: {}'.format(speed))
            self.get_logger().info('Trying to decrease the speed...')
            self.decrease_speed(msg)

    def decrease_speed(self, msg):
        # read the current velocity
        curr_linear = msg.twist.twist.linear.x
        curr_angular = msg.twist.twist.angular.z

        # prepare the message to be published
        published_msg = Twist()
        published_msg.linear.x = msg.twist.twist.linear.x
        published_msg.linear.y = msg.twist.twist.linear.y

        # movement can only be in x-axis or z-axis (exclusive)
        if abs(curr_linear) > self.MAX_SPEED:
            published_msg.linear.x = 5.0 if curr_linear > 0 else -5.0
        elif abs(curr_angular) > self.MAX_SPEED:
            published_msg.angular.z = 5.0 if curr_angular > 0 else -5.0

        self.publisher_.publish(published_msg)


if __name__ == '__main__':
    rclpy.init()
    node = Alert()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
