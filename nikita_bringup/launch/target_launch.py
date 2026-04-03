import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    enable_nav_arg = DeclareLaunchArgument(
        'enable_navigation', default_value='false',
        description='Launch nikita_navigation (head-sweep scan + reactive nav)')
    ld.add_action(enable_nav_arg)

    # communication_launch.py not yet correctly installed
    node_communication = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("nikita_communication"), '/launch', '/communication_launch.py'])
        )
    ld.add_action(node_communication)

    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource([
        FindPackageShare("nikita_hmi"), '/launch', '/hmi_launch.py'])))

    # ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource([
    #     FindPackageShare("nikita_servo_controller"), '/launch', '/servo_controller_launch.py'])))
    
    ld.add_action(IncludeLaunchDescription(AnyLaunchDescriptionSource([
        FindPackageShare("nikita_movement"), '/launch', '/movement_launch.py'])))

    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource([
        FindPackageShare("nikita_teleop"), '/launch', '/teleop_launch.py'])))

    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource([
        FindPackageShare("nikita_lidar"), '/launch', '/lidar_launch.py'])))

    # BNO055 IMU via official ros-jazzy-bno055 package (I2C)
    bno055_config = os.path.join(
        get_package_share_directory('nikita_bringup'), 'config', 'bno055_params.yaml')
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        output='screen',
        parameters=[bno055_config],
    )
    ld.add_action(bno055_node)

    # delay brain launch
    # - the HMI node needs to be started first (it needs the servo voltage released by the relay. The relay is controlled by the HMI node)
    # - the servo node needs to be started first (the servo status is needed by the brain node)
    delay_brain_launch = launch.actions.TimerAction(period=5.0, actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource([
        FindPackageShare("nikita_brain"), '/launch', '/brain_launch.py']))])

    ld.add_action(delay_brain_launch)

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("nikita_navigation"), '/launch', '/navigation_launch.py']),
        condition=IfCondition(LaunchConfiguration('enable_navigation')),
    )
    ld.add_action(navigation_launch)

    return ld

