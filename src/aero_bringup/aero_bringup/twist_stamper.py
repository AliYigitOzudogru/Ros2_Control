import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header

class TwistToTwistStamped(Node):
    def __init__(self):
        super().__init__('twist_to_twiststamped')

        # Enable sim time
        # self.declare_parameter('use_sim_time', True)
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        self.pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)

    def callback(self, msg: Twist):
        stamped = TwistStamped()
        # Use node's clock (sim or real time)
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.twist = msg
        self.pub.publish(stamped)

def main():
    rclpy.init()
    node = TwistToTwistStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
