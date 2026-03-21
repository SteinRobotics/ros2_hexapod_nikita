#!/usr/bin/env python3
"""Bridge between movement node's JointState output and Gazebo's ForwardPositionController.

Subscribes to 'target_joint_states' (sensor_msgs/JointState) published by the
movement node (remapped from 'joint_states') and republishes the positions as
Float64MultiArray on 'forward_position_controller/commands' in the joint order
expected by the controller config.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Joint order must match nikita_gazebo/config/joint_controllers.yaml
CONTROLLER_JOINT_ORDER = [
    'right_front_coxa_joint',
    'right_front_femur_joint',
    'right_front_tibia_joint',
    'right_mid_coxa_joint',
    'right_mid_femur_joint',
    'right_mid_tibia_joint',
    'right_back_coxa_joint',
    'right_back_femur_joint',
    'right_back_tibia_joint',
    'left_front_coxa_joint',
    'left_front_femur_joint',
    'left_front_tibia_joint',
    'left_mid_coxa_joint',
    'left_mid_femur_joint',
    'left_mid_tibia_joint',
    'left_back_coxa_joint',
    'left_back_femur_joint',
    'left_back_tibia_joint',
    'head_yaw_joint',
    'head_pitch_joint',
]


class JointStateBridge(Node):
    def __init__(self):
        super().__init__('joint_state_bridge')
        self.sub_ = self.create_subscription(
            JointState, 'target_joint_states', self.on_joint_states, 10
        )
        self.pub_ = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10
        )

    def on_joint_states(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        out = Float64MultiArray()
        out.data = [name_to_pos.get(j, 0.0) for j in CONTROLLER_JOINT_ORDER]
        self.pub_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
