#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__copyright__ = "Copyright (C) 2025 Christian Stein"
__license__ = "MIT"

import os
# Hide the pygame support prompt
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
import pygame
import threading

import rclpy
from rclpy.node import Node
from nikita_interfaces.msg import JoystickRequest

INTERVAL = 100  # milliseconds, how often the joystick data is published

class NodeTeleop(Node):
    def __init__(self):
        super().__init__('node_teleop')
        self.get_logger().info('starting node_teleop')
        self.msg = JoystickRequest()

       # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick detected! Please connect one.")
            exit()

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Connected to Joystick: {self.joystick.get_name()}")


        self.pub = self.create_publisher(JoystickRequest, 'joystick_request', 10)

        self.running = True
        # Start the pygame event loop in a separate thread.
        self.event_thread = threading.Thread(target=self.event_loop, daemon=True)
        self.event_thread.start()

    def event_loop(self):
        while self.running and rclpy.ok():
            for event in pygame.event.get():
                self.handle_event(event)
            pygame.time.wait(INTERVAL)


    def handle_event(self, event):
        # You can add filtering for different event types.
        if not event.type in (pygame.JOYBUTTONDOWN,
                          pygame.JOYAXISMOTION,
                          pygame.JOYHATMOTION):
            return
        self.publish_joystick_data()

    def publish_joystick_data(self):
        # self.get_logger().info(f"publish_joystick_data")
        pygame.event.pump()  # Process events

        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
        hat_values = self.joystick.get_hat(0)  # Usually only one hat switch exists
        #self.get_logger().info(f"Published: Axes {axes} Buttons {buttons} Hat {hat_values}")

        self.msg.button_a = bool(buttons[2])
        self.msg.button_b = bool(buttons[1])
        self.msg.button_x = bool(buttons[3])
        self.msg.button_y = bool(buttons[0])
        self.msg.button_l1 = bool(buttons[4])
        self.msg.button_l2 = bool(buttons[6])
        self.msg.button_r1 = bool(buttons[5])
        self.msg.button_r2 = bool(buttons[7])
        self.msg.button_select = bool(buttons[8])
        self.msg.button_start = bool(buttons[9])

        if abs(axes[0]) < 0.004:
            self.msg.left_stick_horizontal = 0.0
        else:
            self.msg.left_stick_horizontal = axes[0]

        if abs(axes[1]) < 0.004:
            self.msg.left_stick_vertical = 0.0
        else:
            self.msg.left_stick_vertical = axes[1]

        if abs(axes[2]) < 0.004:
            self.msg.right_stick_horizontal = 0.0
        else:
            self.msg.right_stick_horizontal = axes[2]

        if abs(axes[3]) < 0.004:
            self.msg.right_stick_vertical = 0.0
        else:
            self.msg.right_stick_vertical = axes[3]

        self.msg.dpad_horizontal = hat_values[0]
        self.msg.dpad_vertical = hat_values[1]

        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)

    def shutdown(self):
        # Signal the event loop thread to stop and quit pygame.
        self.running = False
        pygame.quit()

def main(args=None):
    rclpy.init(args=args)
    node = NodeTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down...")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
