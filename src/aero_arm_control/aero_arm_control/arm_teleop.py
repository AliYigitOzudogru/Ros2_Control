import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PointStamped
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
import xacro, tempfile, os
from ikpy.chain import Chain
import numpy as np
from ament_index_python.packages import get_package_share_path
import sys, termios, tty, select


def getch():
    """Reads a single character from terminal (blocking)."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')
        
        # Parse URDF from xacro
        xacro_path = os.path.join(get_package_share_path('aero_description'), 'urdf', 'arm', 'arm.urdf.xacro')
        urdf_file = tempfile.NamedTemporaryFile(delete=False, suffix=".urdf")
        urdf_file.write(xacro.process_file(xacro_path).toxml().encode())
        urdf_file.close()
        
        self.chain = Chain.from_urdf_file(urdf_file.name, active_links_mask=[False, True, True, True, True, False, False])
        
        # Initial target = current end effector pos
        fk_init = self.chain.forward_kinematics([0]*len(self.chain.links))
        self.target = np.array(fk_init[:3, 3])
        
        # Publishers / clients
        self.target_pub = self.create_publisher(PointStamped, "/arm_target", 10)
        self.joint_states = None
        self.sub = self.create_subscription(JointState, "/joint_states", self.js_cb, 10)
        self.client = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        
        # Timer loop
        # self.create_timer(0.05, self.loop)
        while rclpy.ok():
            self.loop()
        self.key_state = set()
        # listener = kb.Listener(on_press=self.on_press, on_release=self.on_release)
        # listener.start()

        print("  ")
        print(" A/D: left/right ")
        print(" W/S: forward/back ")
        print(" Q/E: up/down")
        print(" R: reset target point")
        print(" C: terminate")
        print("  ")
    

    def current_end_effector_position(self):
        fk_init = self.chain.forward_kinematics([0]*len(self.chain.links))
        return np.array(fk_init[:3, 3])

    def js_cb(self, msg):
        self.joint_states = msg.position

    def loop(self):
        # Update target with keyboard input
        step = 0.02

        ch = getch()
        if 'r' == ch: self.target = self.current_end_effector_position()
        if 'w' == ch: self.target[0] += step
        if 's' == ch: self.target[0] -= step
        if 'a' == ch: self.target[2] += step
        if 'd' == ch: self.target[2] -= step
        if 'q' == ch: self.target[1] -= step
        if 'e' == ch: self.target[1] += step
        if 'c' == ch: sys.exit(0)
        
        # Publish target for visualization
        msg = PointStamped()
        # msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.stamp = rclpy.time.Time(seconds=0).to_msg()  # always latest transform
        msg.header.frame_id = "arm_base_link"
        msg.point.x, msg.point.y, msg.point.z = self.target
        self.target_pub.publish(msg)
        
        # Solve IK
        # target_frame = np.eye(4)
        # target_frame[:3, 3] = self.target
        ik_solution = self.chain.inverse_kinematics(target_position=self.target)
        joint_positions = ik_solution [1:5]  # skip fixed base, take 4 revolutes
        
        # Send trajectory goal
        traj = JointTrajectory()
        traj.joint_names = ["arm_joint0", "arm_joint1", "arm_joint2", "arm_joint3"]
        point = JointTrajectoryPoint()
        point.positions = joint_positions.tolist()
        point.time_from_start.sec = 1
        traj.points.append(point)
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj
        if not self.client.wait_for_server(timeout_sec=0.1):
            return
        self.client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    node = ArmTeleop()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
