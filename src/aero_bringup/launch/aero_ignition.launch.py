"""
Ignition Gazebo + PS4 Controller Launch File
Rover'ı Ignition Gazebo'da PS4 kolu ile sürmek için
"""
from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess, SetEnvironmentVariable, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_path
import xacro

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
    
    # Plugin path for Gazebo Sim / Ignition - ONLY gz_ros2_control, not gazebo_ros2_control
    gz_plugin_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib:' + os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
    )
    gz_sim_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib:' + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    )
    # LD_LIBRARY_PATH for plugin dependencies
    ld_library_path = SetEnvironmentVariable(
        name='LD_LIBRARY_PATH',
        value='/opt/ros/humble/lib:' + os.environ.get('LD_LIBRARY_PATH', '')
    )

    # URDF with Ignition enabled - PROCESS IMMEDIATELY to avoid lazy evaluation issues
    robot_description_content = xacro.process_file(
        urdf_path,
        mappings={
            'use_ignition': 'true',
            'use_gazebo': 'false',
            'use_webots': 'false',
            'arm_enabled': 'true'
        }
    ).toxml()
    
    # Store the raw XML string for direct injection
    robot_desc_xml = robot_description_content
    
    # Create parameter value for robot_state_publisher
    robot_description = {'robot_description': robot_description_content}
    
    # Log successful Xacro processing
    xacro_log = LogInfo(msg=f"✓ Xacro processed successfully: {len(robot_desc_xml)} characters")

    # Robot State Publisher - publishes robot_description to topic as well
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            robot_description,
            {'use_sim_time': True, 'publish_frequency': 30.0}
        ],
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

    # Spawn robot - DIRECT STRING INJECTION to bypass topic-based race condition
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', ROBOT,
            '-string', robot_desc_xml,  # Direct XML injection - bypasses /robot_description topic
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        parameters=[
            {'use_sim_time': True},
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
            'max_linear_speed': 3.0,  # Makul hız: 3 m/s = 10.8 km/h
            'max_angular_speed': 2.0,  # Makul açısal hız
            'deadzone': 0.1,
            'arm_speed_scale': 1.0,
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

    # Delayed spawners - robot_state_publisher must be ready before spawn
    delayed_spawn = TimerAction(period=5.0, actions=[spawn_entity])  # Increased delay
    delayed_bridge = TimerAction(period=2.0, actions=[gz_bridge])
    delayed_joy = TimerAction(period=8.0, actions=[joy_node])
    delayed_ps4 = TimerAction(period=9.0, actions=[ps4_controller])
    delayed_joint_state_spawner = TimerAction(period=12.0, actions=[joint_state_broadcaster_spawner])
    delayed_diff_drive_spawner = TimerAction(period=13.0, actions=[diff_drive_controller_spawner])
    delayed_arm_controller_spawner = TimerAction(period=14.0, actions=[arm_controller_spawner])

    return LaunchDescription([
        gz_resource_path,
        gz_plugin_path,
        gz_sim_plugin_path,
        ld_library_path,
        xacro_log,  # Log Xacro processing success
        robot_state_publisher,
        ignition_gazebo,
        delayed_bridge,
        delayed_spawn,
        delayed_joy,
        delayed_ps4,
        delayed_joint_state_spawner,
        delayed_diff_drive_spawner,
        delayed_arm_controller_spawner,
    ])
