#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PS4ArmTeleop(Node):
    def __init__(self):
        super().__init__('ps4_arm_teleop')
        # Based on your joystick output: right stick horizontal=3, vertical=4
        self.declare_parameter('axis_base', 3)
        self.declare_parameter('axis_shoulder', 4)
        # Use the same joint names as controller config
        self.declare_parameter('joint_names', ['arm_joint0', 'arm_joint1', 'arm_joint2', 'arm_joint3'])
        self.declare_parameter('topic', '/arm_controller/joint_trajectory')
        self.declare_parameter('scale', 0.5)

        self.axis_base = self.get_parameter('axis_base').value
        self.axis_shoulder = self.get_parameter('axis_shoulder').value
        self.joint_names = self.get_parameter('joint_names').value
        self.topic = self.get_parameter('topic').value
        self.scale = self.get_parameter('scale').value

        # internal positions (match number of joints)
        self.positions = [0.0 for _ in self.joint_names]

        self.pub = self.create_publisher(JointTrajectory, self.topic, 10)
        self.sub = self.create_subscription(Joy, 'joy', self.joy_cb, 10)

    def joy_cb(self, msg: Joy):
        base = msg.axes[self.axis_base] if len(msg.axes) > self.axis_base else 0.0
        sh = msg.axes[self.axis_shoulder] if len(msg.axes) > self.axis_shoulder else 0.0

        # integrate small deltas: map base->joint0, shoulder->joint1
        dt = 0.05
        if len(self.positions) >= 1:
            self.positions[0] += base * self.scale * dt
        if len(self.positions) >= 2:
            self.positions[1] += sh * self.scale * dt

        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        p = JointTrajectoryPoint()
        p.positions = self.positions
        p.time_from_start.sec = 0
        p.time_from_start.nanosec = int(dt * 1e9)
        traj.points = [p]

        self.pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = PS4ArmTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
