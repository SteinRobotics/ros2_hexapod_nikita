import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('nikita_servo_controller'),
        'config',
        'servo_description.yaml'
        )
        
    node=Node(
        package = 'nikita_servo_controller',
        name = 'node_servo_controller',
        executable = 'node_servo_controller',
        output="screen",
        parameters = [config]
    )
    ld.add_action(node)
    return ld
