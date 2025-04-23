import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
import launch_ros
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
import random


def generate_launch_description():
    ####### DATA INPUT ##########
    xacro_file_name = "project2_pkg.urdf.xacro"
    package_description = "project2_pkg"

    # Position and orientation
    position = [0.0, -0.5, 0.5]
    orientation = [0.0, 0.0, 1.57]
    robot_base_name = "chassi"
    ####### DATA INPUT END ##########

    # Get path to relevant files
    pkg_share = get_package_share_directory(package_description)
    urdf_dir = os.path.join(pkg_share, 'urdf')
    xacro_file = os.path.join(urdf_dir, xacro_file_name)
    controller_config = os.path.join(pkg_share, 'config', 'control.yaml')


    # Process xacro with Command substitution, passing the controller config path
    # (This part remains correct, ensuring the plugin finds the YAML file)
    robot_description_content = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' controller_config_path:=', controller_config 
        ]),
        value_type=str
    )

    # Entity name
    entity_name = f"{robot_base_name}-{random.random()}"

    # Spawn robot node
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', entity_name,
            '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),
            '-R', str(orientation[0]), '-P', str(orientation[1]), '-Y', str(orientation[2]),
            '-topic', '/robot_description'
        ]
    )

    # Simulation time configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot State Publisher with modified parameters
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }],
        output="screen"
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    # Controller spawner nodes - *** REMOVED PARAMETER ARGUMENTS ***
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        # Only specify the controller name and manager service
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"], 
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
         # Only specify the controller name and manager service
        arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
         # Only specify the controller name and manager service
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )

    # Event handlers for delayed controller starts (Keep as is)
    delay_controllers = [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster_spawner]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[velocity_controller_spawner, position_controller_spawner]
            )
        )
    ]

    # TF node (Keep as is)
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['1', '0', '0', '0', '0', '0', '1', '/map', '/dummy_link'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='Enable joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Use simulation time'
        ),
        joint_state_publisher_node,
        robot_state_publisher,
        spawn_robot,
        tf_node,
        *delay_controllers
    ])