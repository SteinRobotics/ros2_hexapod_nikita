"""Launch file to display the Nikita hexapod URDF in RViz with joint_state_publisher_gui."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


# Snap-based VS Code sets GTK/GIO env vars that crash RViz2 and other GUI
# processes (symbol lookup error in snap's libpthread).  Clear them so that
# the system-installed libraries are used instead.
_SNAP_GUI_OVERRIDES = {
    k: '' for k in (
        'GTK_PATH', 'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE',
        'GIO_MODULE_DIR', 'GSETTINGS_SCHEMA_DIR',
    ) if k in os.environ
}


def generate_launch_description():
    pkg_description = get_package_share_directory('nikita_description')

    xacro_file = os.path.join(pkg_description, 'urdf', 'nikita.urdf.xacro')
    robot_description = xacro.process_file(xacro_file, mappings={'use_sim': 'false'}).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        additional_env=_SNAP_GUI_OVERRIDES,
    )

    rviz_config = os.path.join(pkg_description, 'rviz', 'model.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        additional_env=_SNAP_GUI_OVERRIDES,
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
