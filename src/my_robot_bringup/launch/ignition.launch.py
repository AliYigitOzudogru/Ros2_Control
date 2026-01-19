from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node 
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
import os
from ament_index_python.packages import get_package_share_path
import xacro

from controller_manager import spawner

ROBOT = 'my_robot'

def launch_setup(context):

    urdf_path = os.path.join(get_package_share_path(f'{ROBOT}_description'),
                             'urdf', f'{ROBOT}.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_path(f'{ROBOT}_description'),
                                    'rviz', 'urdf_config.rviz')
    
    controller_manager_config_path = os.path.join(get_package_share_path(f'{ROBOT}_bringup'),
                                                  'config', f'{ROBOT}_controllers.yaml')
    
    ignition_config_path = os.path.join(get_package_share_path(f'{ROBOT}_bringup'), 'config', 'ignition.yaml')
    
    
    use_ignition = "true"
    use_gazebo = "false"

    robot_description = xacro.process_file(urdf_path, mappings={"use_gazebo": use_gazebo, "use_ignition": use_ignition}).toxml()
    # robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description}, controller_manager_config_path]
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['joint_state_broadcaster']
    )

    # ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true
    ddc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['diff_drive_controller']
    )

    jtc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['arm_controller']
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description"]
    )

    to_launch = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py"
                ])
            ),
            launch_arguments=[("gz_args", "-r -v 1 empty.sdf")] # -r
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[jsb_spawner]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=jsb_spawner,
                on_exit=[jtc_spawner]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=jtc_spawner,
                on_exit=[ddc_spawner]
            )
        ), # ddc
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[{"config_file": ignition_config_path}]
        ),
        gz_spawn_entity,
        robot_state_publisher_node,
        rviz2_node
    ]

    return to_launch

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])