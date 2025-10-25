import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config_dir = os.path.join(
        get_package_share_directory('nikita_movement'),
        'config'
    )

    anatomy = os.path.join(config_dir, 'anatomy.yaml')
    servo_description = os.path.join(config_dir, 'servo_description.yaml')

    node = Node(
        package = 'nikita_movement',
        name = 'node_movement',
        executable = 'node_movement',
        output = 'screen',
        # load the anatomy config and set the controller to offline (use mock)
        parameters = [anatomy, servo_description, {'SERVO_CONTROLLER_OFFLINE': True}]
    )
    ld.add_action(node)
    return ld
