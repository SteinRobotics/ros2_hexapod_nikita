"""Launch Gazebo Harmonic simulation of the Nikita hexapod with ros2_control."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    pkg_description = get_package_share_directory('nikita_description')
    pkg_gazebo = get_package_share_directory('nikita_gazebo')

    # Process XACRO with simulation enabled
    xacro_file = os.path.join(pkg_description, 'urdf', 'nikita.urdf.xacro')
    robot_description = xacro.process_file(xacro_file, mappings={'use_sim': 'true'}).toxml()

    world_file = os.path.join(pkg_gazebo, 'worlds', 'empty.sdf')

    # --- Launch Gazebo Harmonic ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            ])
        ),
        launch_arguments={'gz_args': ['-r ', world_file]}.items(),
    )

    # --- Robot State Publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    # --- Spawn the robot into Gazebo ---
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'nikita',
            '-z', '0.07',  # spawn slightly above ground (standing height ~0.05m)
        ],
        output='screen',
    )

    # --- ros2_control: activate controllers after spawn ---
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller'],
    )

    # Chain: spawn entity → start joint_state_broadcaster → start position controller
    activate_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    activate_position_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[forward_position_controller_spawner],
        )
    )

    # --- Bridge: forward /clock from Gazebo to ROS ---
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        activate_joint_state_broadcaster,
        activate_position_controller,
        gz_ros_bridge,
    ])
