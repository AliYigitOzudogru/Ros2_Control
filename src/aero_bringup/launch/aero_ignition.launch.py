"""
Ignition Gazebo + PS4 Controller Launch File
Rover'ı Ignition Gazebo'da PS4 kolu ile sürmek için
"""
from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_path

ROBOT = 'aero'

def generate_launch_description():

    pkg_share = get_package_share_path(f'{ROBOT}_description')
    bringup_share = get_package_share_path(f'{ROBOT}_bringup')
    
    urdf_path = os.path.join(pkg_share, 'urdf', f'{ROBOT}.urdf.xacro')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf_config.rviz')
    controller_config = os.path.join(bringup_share, 'config', 'controllers.yaml')

    # Environment variables for Ignition to find plugins and meshes
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_share, 'meshes') + ':' + str(pkg_share)
    )
    
    # Plugin path for Gazebo Sim / Ignition
    gz_plugin_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib'
    )
    gz_sim_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib'
    )

    # URDF with Ignition enabled
    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_path,
            ' use_ignition:=true',
            ' use_gazebo:=false',
            ' use_webots:=false',
            ' arm_enabled:=false'
        ]),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # Ignition Gazebo
    ignition_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', '-r', 'empty.sdf'],
        output='screen',
        additional_env={
            'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': '/opt/ros/humble/lib',
            'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/humble/lib'
        }
    )

    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', ROBOT,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    # Bridge for clock
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
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
            'arm_speed_scale': 1.0,
        }],
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

    # Delayed spawners
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_entity])
    delayed_bridge = TimerAction(period=2.0, actions=[gz_bridge])
    delayed_joy = TimerAction(period=5.0, actions=[joy_node])
    delayed_ps4 = TimerAction(period=6.0, actions=[ps4_controller])
    delayed_joint_state_spawner = TimerAction(period=8.0, actions=[joint_state_broadcaster_spawner])
    delayed_diff_drive_spawner = TimerAction(period=9.0, actions=[diff_drive_controller_spawner])

    return LaunchDescription([
        gz_resource_path,
        gz_plugin_path,
        gz_sim_plugin_path,
        robot_state_publisher,
        ignition_gazebo,
        delayed_bridge,
        delayed_spawn,
        delayed_joy,
        delayed_ps4,
        delayed_joint_state_spawner,
        delayed_diff_drive_spawner,
    ])
