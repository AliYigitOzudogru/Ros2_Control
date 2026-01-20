from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([
        get_package_share_directory('my_robot_bringup'),
        'config',
        'ps4_teleop.yaml'
    ])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[config],
        remappings=[('cmd_vel', '/cmd_vel')],
        output='screen'
    )

    # Custom rover teleop (R2 throttle + left stick steering)
    rover_teleop = Node(
        package='my_robot_bringup',
        executable='ps4_rover_teleop.py',
        name='ps4_rover_teleop',
        output='screen',
        parameters=[{
            'throttle_axis': 5,
            'steer_axis': 0,
            'throttle_scale': 1.0,
            'steer_scale': 1.0,
            'deadzone': 0.05,
            'topic': '/cmd_vel'
        }]
    )

    # Arm teleop (right stick -> arm joints)
    arm_teleop = Node(
        package='my_robot_bringup',
        executable='ps4_arm_teleop.py',
        name='ps4_arm_teleop',
        output='screen',
        parameters=[{
            'axis_base': 2,
            'axis_shoulder': 3,
            'joint_names': ['joint1', 'joint2'],
            'topic': '/arm_controller/joint_trajectory',
            'scale': 0.5
        }]
    )

    return LaunchDescription([joy_node, teleop_node, rover_teleop, arm_teleop])
