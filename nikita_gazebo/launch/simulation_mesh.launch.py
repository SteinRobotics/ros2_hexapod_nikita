"""Launch the STL-based Nikita model in Gazebo Harmonic."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    description_share = get_package_share_directory('nikita_description')
    gazebo_share = get_package_share_directory('nikita_gazebo')
    robot_description = xacro.process_file(
        os.path.join(description_share, 'urdf', 'nikita_mesh.urdf.xacro'),
        mappings={'use_sim': 'true'},
    ).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': ['-r ', os.path.join(gazebo_share, 'worlds', 'empty.sdf')]}.items(),
    )
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}])
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-topic', 'robot_description', '-name', 'nikita_mesh', '-z', '0.07'],
                 output='screen')
    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster'], output='screen')
    position = Node(package='controller_manager', executable='spawner',
                    arguments=['forward_position_controller'], output='screen')
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )
    return LaunchDescription([
        gazebo, rsp, spawn, clock_bridge,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[position])),
    ])
