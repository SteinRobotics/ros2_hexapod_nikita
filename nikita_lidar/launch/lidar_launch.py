from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node=Node(
        package = 'nikita_lidar',
        name = 'node_lidar',
        executable = 'node_lidar',
        output="screen",
    )
    ld.add_action(node)
    return ld
