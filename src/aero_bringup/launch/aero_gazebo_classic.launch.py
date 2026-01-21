"""
Gazebo Classic + PS4 Controller Launch File
Rover'ı Gazebo Classic'te PS4 kolu ile sürmek için
"""
from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_path
import xacro

ROBOT = 'aero'

def generate_launch_description():

    pkg_share = get_package_share_path(f'{ROBOT}_description')
    bringup_share = get_package_share_path(f'{ROBOT}_bringup')
    
    urdf_path = os.path.join(pkg_share, 'urdf', f'{ROBOT}.urdf.xacro')
    controller_config = os.path.join(bringup_share, 'config', 'controllers.yaml')
    world_file = os.path.join(bringup_share, 'worlds', 'empty.world')

    # URDF - Gazebo Classic enabled
    robot_description_content = xacro.process_file(
        urdf_path,
        mappings={
            'use_ignition': 'false',
            'use_gazebo': 'true',
            'use_webots': 'false',
            'arm_enabled': 'true'
        }
    ).toxml()
    
    robot_description = {'robot_description': robot_description_content}

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            robot_description,
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # Gazebo Classic
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', ROBOT,
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    # Joy Node (PS4 Controller)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )

    # PS4 Rover Controller
    ps4_controller = Node(
        package='aero_arm_control',
        executable='ps4_rover_controller',
        name='ps4_rover_controller',
        parameters=[{
            'use_sim_time': True,
            'max_linear_speed': 2.0,
            'max_angular_speed': 2.0,
            'deadzone': 0.1,
        }],
        remappings=[
            ('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped'),
        ],
        output='screen',
        emulate_tty=True,
    )

    # Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Delayed actions
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_entity])
    delayed_joy = TimerAction(period=6.0, actions=[joy_node])
    delayed_ps4 = TimerAction(period=7.0, actions=[ps4_controller])
    delayed_jsb = TimerAction(period=8.0, actions=[joint_state_broadcaster_spawner])
    delayed_ddc = TimerAction(period=10.0, actions=[diff_drive_controller_spawner])
    delayed_arm = TimerAction(period=12.0, actions=[arm_controller_spawner])

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        delayed_spawn,
        delayed_joy,
        delayed_ps4,
        delayed_jsb,
        delayed_ddc,
        delayed_arm,
    ])
