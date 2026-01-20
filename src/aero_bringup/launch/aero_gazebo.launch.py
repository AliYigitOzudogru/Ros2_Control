from launch import LaunchDescription
from launch_ros.actions import Node 
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_path

ROBOT = 'aero'

def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path(f'{ROBOT}_description'),
                             'urdf', f'{ROBOT}.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_path(f'{ROBOT}_description'),
                                    'rviz', 'urdf_config.rviz')
    
    controller_manager_config_path = os.path.join(get_package_share_path(f'{ROBOT}_bringup'),
                                                  'config', 'controllers.yaml')
    
    # World file (empty world)
    world_path = os.path.join(get_package_share_path('gazebo_ros'), 'worlds', 'empty.world')

    # URDF with Gazebo enabled
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path, ' use_gazebo:=true']),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # Start Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen'
    )

    # Start Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", ROBOT,
            "-x", "0.0",
            "-y", "0.0", 
            "-z", "0.5"
        ],
        output='screen'
    )

    # Joint State Broadcaster Spawner
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}]
    )

    # Diff Drive Controller Spawner
    ddc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}]
    )

    # Arm Controller Spawner  
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}]
    )

    # Delayed spawners
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )

    delayed_jsb = TimerAction(
        period=10.0,
        actions=[jsb_spawner]
    )

    delayed_ddc = TimerAction(
        period=12.0,
        actions=[ddc_spawner]
    )

    delayed_arm = TimerAction(
        period=14.0,
        actions=[arm_spawner]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gzserver,
        gzclient,
        delayed_spawn,
        delayed_jsb,
        delayed_ddc,
        delayed_arm,
    ])
