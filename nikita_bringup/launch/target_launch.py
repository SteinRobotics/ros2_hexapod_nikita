import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # communication_launch.py not yet correctly installed
    node_communication = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("nikita_communication"), '/launch', '/communication_launch.py'])
        )
    ld.add_action(node_communication)

    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource([
        FindPackageShare("nikita_hmi"), '/launch', '/hmi_launch.py'])))

    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource([
        FindPackageShare("nikita_servo_controller"), '/launch', '/servo_controller_launch.py'])))
    
    ld.add_action(IncludeLaunchDescription(AnyLaunchDescriptionSource([
        FindPackageShare("nikita_movement"), '/launch', '/movement_launch.py'])))

    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource([
        FindPackageShare("nikita_teleop"), '/launch', '/teleop_launch.py'])))

    # delay brain launch
    # - the HMI node needs to be started first (it needs the servo voltage released by the relay. The relay is controlled by the HMI node)
    # - the servo node needs to be started first (the servo status is needed by the brain node)
    delay_brain_launch = launch.actions.TimerAction(period=5.0, actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource([
        FindPackageShare("nikita_brain"), '/launch', '/brain_launch.py']))])

    ld.add_action(delay_brain_launch)

    return ld

