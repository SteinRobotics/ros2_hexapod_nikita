"""Display the STL-based Nikita model in RViz."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    share = get_package_share_directory('nikita_description')
    description = xacro.process_file(
        os.path.join(share, 'urdf', 'nikita_mesh.urdf.xacro'),
        mappings={'use_sim': 'false'},
    ).toxml()
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'model.rviz')]),
    ])
