import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('nikita_movement'),
        'config',
        'anatomy.yaml'
        )

    node=Node(
        package = 'nikita_movement',
        name = 'node_movement',
        executable = 'node_movement',
        output="screen",
        parameters = [config]
    )
    ld.add_action(node)
    return ld
