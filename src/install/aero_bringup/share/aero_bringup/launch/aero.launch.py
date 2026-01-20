from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node 
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_path
import xacro

from controller_manager import spawner

ROBOT = 'aero'

def launch_setup(context):

    urdf_path = os.path.join(get_package_share_path(f'{ROBOT}_description'),
                             'urdf', f'{ROBOT}.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_path(f'{ROBOT}_description'),
                                    'rviz', 'urdf_config.rviz')
    
    controller_manager_config_path = os.path.join(get_package_share_path(f'{ROBOT}_bringup'),
                                                  'config', 'controllers.yaml')
    
    
    use_ignition = LaunchConfiguration("use_ignition", default="false").perform(context)
    use_gazebo = LaunchConfiguration("use_gazebo", default="false").perform(context)

    if use_ignition == 'true':
        print("Use the other launch file.")
        return

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

    # --- JOYSTICK NODE ---

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',  # PS4 kolu
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )

    # --- PS4 CONTROLLER NODE ---
    ps4_controller_node = Node(
        package='aero_arm_control',
        executable='ps4_rover_controller',
        name='ps4_rover_controller',
        parameters=[{
            'use_sim_time': LaunchConfiguration("use_gazebo"),
            'max_linear_speed': 2.0,
            'max_angular_speed': 2.0,
            'deadzone': 0.1,
            'arm_speed_scale': 1.0,
        }],
        output='screen',
        emulate_tty=True,
    )


    to_launch = [
        robot_state_publisher_node,
        controller_manager,
        jsb_spawner,
        ddc_spawner,
        jtc_spawner,
        rviz2_node,
        ps4_controller_node
    ]

    if use_gazebo == "true":
        # in the works
        to_launch.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("gazebo_ros"),
                        "launch",
                        "gazebo.launch.py"
                    ])
                )
            )
        )

        to_launch.append(Node(
            package="ros_gazebo",
            executable="spawn_entity.py",
            arguments=["-topic", "robot_description", "-entity", ROBOT]
        ))
    
    return to_launch

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_ignition", default_value="false"),
        DeclareLaunchArgument("use_gazebo", default_value="false"),
        OpaqueFunction(function=launch_setup)
    ])
