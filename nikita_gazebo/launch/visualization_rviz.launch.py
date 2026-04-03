"""Launch RViz visualization with movement pipeline and teleop_twist_keyboard for cmd_vel input."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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

    # Process XACRO (no Gazebo plugins needed for RViz-only)
    xacro_file = os.path.join(pkg_description, 'urdf', 'nikita.urdf.xacro')
    robot_description = xacro.process_file(xacro_file, mappings={'use_sim': 'false'}).toxml()

    # --- Robot State Publisher (TF from joint_states) ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    # --- Joint State Publisher (provides default joint values at startup,
    #     merges movement node output via source_list) ---
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'source_list': ['target_joint_states']}],
    )

    # --- Movement node (offline / no hardware) ---
    # Publishes to target_joint_states; joint_state_publisher merges into joint_states
    pkg_movement = get_package_share_directory('nikita_movement')
    movement_config_dir = os.path.join(pkg_movement, 'config')
    movement_node = Node(
        package='nikita_movement',
        name='node_movement',
        executable='node_movement',
        output='screen',
        parameters=[
            os.path.join(movement_config_dir, 'anatomy.yaml'),
            os.path.join(movement_config_dir, 'servo_description.yaml'),
            {'SERVO_CONTROLLER_OFFLINE': True},
        ],
        remappings=[('joint_states', 'target_joint_states')],
    )

    # --- Brain node (routes cmd_vel → cmd_movement) ---
    pkg_brain = get_package_share_directory('nikita_brain')
    brain_config = os.path.join(pkg_brain, 'config', 'parameter.yaml')
    brain_node = Node(
        package='nikita_brain',
        name='node_brain',
        executable='node_brain',
        output='screen',
        parameters=[brain_config],
    )
    
    communication_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nikita_communication'),
                'launch',
                'communication_launch.py',
            ])
        ),
    )

    enable_lidar = LaunchConfiguration('enable_lidar')
    declare_enable_lidar = DeclareLaunchArgument(
        'enable_lidar',
        default_value='false',
        description='Start nikita_lidar node and publish scan_1d for RViz Range display.',
    )
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nikita_lidar'),
                'launch',
                'lidar_launch.py',
            ])
        ),
        condition=IfCondition(enable_lidar),
    )

    # --- RViz ---
    rviz_config = os.path.join(pkg_description, 'rviz', 'model.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        additional_env=_SNAP_GUI_OVERRIDES,
    )

    # --- teleop_twist_keyboard for cmd_vel input ---
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        prefix='xterm -e',
        output='screen',
    )

    # Delay brain + movement so robot_state_publisher is ready
    delayed_nodes = TimerAction(
        period=1.0,
        actions=[movement_node, brain_node, teleop],
    )

    return LaunchDescription([
        declare_enable_lidar,
        robot_state_publisher,
        joint_state_publisher,
        communication_launch,
        lidar_launch,
        rviz,
        delayed_nodes,
    ])
