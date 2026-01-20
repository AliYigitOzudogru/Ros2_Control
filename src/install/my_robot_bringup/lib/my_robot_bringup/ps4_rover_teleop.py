#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class PS4RoverTeleop(Node):
    def __init__(self):
        super().__init__('ps4_rover_teleop')
        self.declare_parameter('throttle_axis', 5)
        self.declare_parameter('steer_axis', 0)
        self.declare_parameter('throttle_scale', 1.0)
        self.declare_parameter('steer_scale', 1.0)
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('topic', '/cmd_vel')

        self.throttle_axis = self.get_parameter('throttle_axis').value
        self.steer_axis = self.get_parameter('steer_axis').value
        self.throttle_scale = self.get_parameter('throttle_scale').value
        self.steer_scale = self.get_parameter('steer_scale').value
        self.deadzone = self.get_parameter('deadzone').value
        self.topic = self.get_parameter('topic').value

        self.pub = self.create_publisher(Twist, self.topic, 10)
        self.sub = self.create_subscription(Joy, 'joy', self.joy_cb, 10)
        # remember trigger rest value on first message to determine convention
        self._trigger_rest = None

    def joy_cb(self, msg: Joy):
        twist = Twist()
        # safe indexing
        ta = msg.axes[self.throttle_axis] if len(msg.axes) > self.throttle_axis else 0.0
        sa = msg.axes[self.steer_axis] if len(msg.axes) > self.steer_axis else 0.0

        # Determine trigger rest convention on first message and use it
        if self._trigger_rest is None:
            self._trigger_rest = ta

        # If rest value is positive (driver reports rest ~ +1), map using
        # (1.0 .. -1.0) -> (0 .. 1). Otherwise use (-1.0 .. 1.0) -> (0 .. 1).
        if self._trigger_rest > 0.0:
            throttle = (1.0 - ta) / 2.0
        else:
            throttle = (ta + 1.0) / 2.0
        # deadzone
        if abs(sa) < self.deadzone:
            sa = 0.0

        twist.linear.x = throttle * float(self.throttle_scale)
        twist.angular.z = sa * float(self.steer_scale)

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PS4RoverTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
