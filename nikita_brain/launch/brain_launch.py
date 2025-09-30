import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('nikita_brain'),
        'config',
        'parameter.yaml'
        )

    node=Node(
        package = 'nikita_brain',
        name = 'node_brain',
        executable = 'node_brain',
        output="screen",
        parameters = [config]
    )
    ld.add_action(node)
    return ld
