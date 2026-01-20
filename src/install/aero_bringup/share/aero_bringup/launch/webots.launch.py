from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
import os
from ament_index_python.packages import get_package_share_path
import xacro
import launch


from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix

from controller_manager import spawner

ROBOT = 'aero'

def launch_setup(context):

    bringup_share_folder = get_package_share_path(f'{ROBOT}_bringup')
    description_share_folder = get_package_share_path(f'{ROBOT}_description')

    urdf_path = os.path.join(description_share_folder, 'urdf', f'{ROBOT}.urdf.xacro')
    rviz_config_path = os.path.join(description_share_folder, 'rviz', 'urdf_config.rviz')
    
    controller_manager_config_path = os.path.join(bringup_share_folder, 'config', 'controllers.yaml')
    ignition_config_path = os.path.join(bringup_share_folder, 'config', 'ignition_bridge.yaml')
    slam_config_path = os.path.join(bringup_share_folder, 'config', 'slam.yaml')
    navigation_config_path = os.path.join(bringup_share_folder, 'config', 'navigation.yaml')

    world_path = os.path.join(bringup_share_folder, 'worlds', 'test.sdf') # 'empty.sdf'
    
    
    use_ignition = "false"
    use_gazebo = "false"
    use_webots = "true"
    use_nav2 = "true"

    robot_description = xacro.process_file(urdf_path, mappings={"use_gazebo": use_gazebo, "use_ignition": use_ignition, "use_webots": use_webots}).toxml()
    # robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    with open("/tmp/aero.urdf", "w", encoding="utf-8") as f:
        f.write(robot_description)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description, "use_sim_time": True}]
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
        arguments=['-d', rviz_config_path],
        parameters=[{"use_sim_time": True}]
    )

    webots = WebotsLauncher(
        world=os.path.join(bringup_share_folder, "worlds", "my_world.wbt"),
        ros2_supervisor=True
    )
    ros2_supervisor = Ros2SupervisorLauncher()
    spawn_entity = Node(
        package='webots_ros2_driver',
        executable='driver',
            parameters=[
                {'robot_description': robot_description},  # Implement this
            ],
        output='screen'
    )

    # Launches the SLAM node for simultaneous localization and mapping.
    # The `online_async_launch.py` is used for live mapping, not pre-recorded data.
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                os.path.join(get_package_share_path("slam_toolbox"), "launch", "online_async_launch.py")
            ])
        ]),
        launch_arguments={
            "slam_params_file": slam_config_path,
            "use_sim_time": "true", # LaunchConfiguration("use_sim_time"),
        }.items()
        # condition=IfCondition(LaunchConfiguration("use_slam")),
    )


    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[os.path.join(bringup_share_folder, 'config', 'ekf.yaml')],
        # remappings=[('odometry/filtered', '/odom')]  # Remap to publish to /odom
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                os.path.join(get_package_share_path("nav2_bringup"), "launch", "navigation_launch.py")
            ])
        ]),
        launch_arguments={
            "params_file": navigation_config_path,
            "use_sim_time": "true" # LaunchConfiguration("use_sim_time")
        }.items()
    )

    to_launch = [
        jsb_spawner,
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
        webots,
        webots._supervisor,
        # spawn_entity,
        # ros2_supervisor,
        robot_state_publisher_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ddc_spawner,
                on_exit=[ekf_node]
            )
        ),
        RegisterEventHandler(
           event_handler=OnProcessExit(
               target_action=ddc_spawner,
               on_exit=[slam_toolbox]
           )
        ),
        RegisterEventHandler(
           event_handler=OnProcessExit(
               target_action=ddc_spawner,
               on_exit=[nav2_launch]
           )
        ),
        controller_manager,
        rviz2_node,
        ExecuteProcess(
            cmd=['python3', '/ros-ws/src/aero_bringup/aero_bringup/twist_stamper.py'],
            output='screen',
            #shell=True
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )

    ]

    if use_nav2:
        pass

    return to_launch

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])