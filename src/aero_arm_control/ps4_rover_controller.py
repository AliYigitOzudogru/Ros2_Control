#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class PS4Controller(Node):
    def __init__(self):
        super().__init__('ps4_rover_controller')
        self.pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel', 10)
        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.max_linear = 2.0
        self.max_angular = 2.0

    def joy_callback(self, msg: Joy):
        twist = Twist()
        twist.linear.x = msg.axes[1] * self.max_linear   # sol stick Y
        twist.angular.z = msg.axes[0] * self.max_angular # sol stick X
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PS4Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
