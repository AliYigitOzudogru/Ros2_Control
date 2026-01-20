#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- Launch argümanları ---
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # --- JOYSTICK NODE ---
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

    # --- PS4 CONTROLLER NODE ---
    ps4_controller_node = Node(
        package='aero_arm_control',
        executable='ps4_rover_controller',
        name='ps4_rover_controller',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_linear_speed': 2.0,
            'max_angular_speed': 2.0,
            'deadzone': 0.1,
            'arm_speed_scale': 1.0,
        }],
        output='screen',
        emulate_tty=True,
    )

    # --- Opsiyonel: aero.launch.py’yi çağır ---
    include_aero_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('aero_bringup'), 'launch', 'aero.launch.py'])
        ])
    )

    return LaunchDescription([
        declare_use_sim_time,
        include_aero_launch,
        joy_node,
        ps4_controller_node,
    ])
