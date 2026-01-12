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

    generic_layout = {}

    # Shanwan Twin Usb Joystick
    generic_layout['Shanwan Twin Usb Joystick'] = {
        'button': {'A': 0, 'B': 1, 'X': 3, 'Y': 4, 'L1': 6, 'R1': 7, 'L2': 8, 'R2': 9, 'SELECT': 10, 'START': 11, 'HOME': 12, 'RETURN': 13, 'LIST': 14}, 
        'axis': {'LEFT_STICK_HORIZONTAL': 0, 'LEFT_STICK_VERTICAL': 1, 'RIGHT_STICK_HORIZONTAL': 2, 'RIGHT_STICK_VERTICAL': 3}, 
        'dpad': {'HORIZONTAL': 0, 'VERTICAL': 1}
    }

    # PS3 look alike
    generic_layout['Twin USB Joystick'] = {
        'button': {'A': 2, 'B': 1, 'X': 3, 'Y': 0, 'L1': 4, 'R1': 5, 'L2': 6, 'R2': 7, 'SELECT': 8, 'START': 9}, 
        'axis': {'LEFT_STICK_HORIZONTAL': 0, 'LEFT_STICK_VERTICAL': 1, 'RIGHT_STICK_HORIZONTAL': 2, 'RIGHT_STICK_VERTICAL': 3}, 
        'dpad': {'HORIZONTAL': 0, 'VERTICAL': 1}
    }

    axis_deadzone = 0.004

    def __init__(self):
        super().__init__('node_teleop')
        self.get_logger().info('starting node_teleop')
        self.msg = JoystickRequest()
        self.start_button_press_time = None  # Track when start button was first pressed

       # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick detected! Please connect one.")
            exit()

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Connected to Joystick: {self.joystick.get_name()}")

        if self.joystick.get_name() not in self.generic_layout:
            self.get_logger().error(f"Joystick layout for '{self.joystick.get_name()}' not found.")
            exit()

        self.layout = self.generic_layout.get(self.joystick.get_name())


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
                            pygame.JOYBUTTONUP,
                          pygame.JOYAXISMOTION,
                          pygame.JOYHATMOTION):
            return
        self.publish_joystick_data()

    def debounce_button(self, button_pressed, press_time_tracker, debounce_duration_s):
        if button_pressed:
            if press_time_tracker is None:
                # Button just pressed, start tracking time
                return False, self.get_clock().now()
            else:
                # Check if button has been held for required duration
                elapsed_s = (self.get_clock().now() - press_time_tracker).nanoseconds / 1e9
                if elapsed_s >= debounce_duration_s:
                    return True, press_time_tracker
                else:
                    return False, press_time_tracker
        else:
            # Button released, reset tracking
            return False, None

    def publish_joystick_data(self):
        # self.get_logger().info(f"publish_joystick_data")
        pygame.event.pump()  # Process events

        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
        hat_values = self.joystick.get_hat(0)  # Usually only one hat switch exists
        #self.get_logger().info(f"Published: Axes {axes} Buttons {buttons} Hat {hat_values}")

        self.msg.button_a = bool(buttons[self.layout['button']['A']])
        self.msg.button_b = bool(buttons[self.layout['button']['B']])
        self.msg.button_x = bool(buttons[self.layout['button']['X']])
        self.msg.button_y = bool(buttons[self.layout['button']['Y']])
        self.msg.button_l1 = bool(buttons[self.layout['button']['L1']])
        self.msg.button_l2 = bool(buttons[self.layout['button']['L2']])
        self.msg.button_r1 = bool(buttons[self.layout['button']['R1']])
        self.msg.button_r2 = bool(buttons[self.layout['button']['R2']])
        self.msg.button_select = bool(buttons[self.layout['button']['SELECT']])

        # Debouncing logic for start button (2 seconds)
        start_button_pressed = bool(buttons[self.layout['button']['START']])
        self.msg.button_start, self.start_button_press_time = self.debounce_button(
            start_button_pressed, self.start_button_press_time, 2.0
        )

        self.msg.button_home = bool(buttons[self.layout['button'].get('HOME', -1)]) if 'HOME' in self.layout['button'] else False
        # self.msg.button_return = bool(buttons[self.layout['button'].get('RETURN', -1)]) if 'RETURN' in self.layout['button'] else False
        # self.msg.button_list = bool(buttons[self.layout['button'].get('LIST', -1)]) if 'LIST' in self.layout['button'] else False
        if abs(axes[0]) < self.axis_deadzone:
            self.msg.left_stick_horizontal = 0.0
        else:
            self.msg.left_stick_horizontal = axes[self.layout['axis']['LEFT_STICK_HORIZONTAL']]

        if abs(axes[1]) < self.axis_deadzone:
            self.msg.left_stick_vertical = 0.0
        else:
            self.msg.left_stick_vertical = -axes[self.layout['axis']['LEFT_STICK_VERTICAL']]  # Invert vertical axis

        if abs(axes[2]) < self.axis_deadzone:
            self.msg.right_stick_horizontal = 0.0
        else:
            self.msg.right_stick_horizontal = axes[self.layout['axis']['RIGHT_STICK_HORIZONTAL']]

        if abs(axes[3]) < self.axis_deadzone:
            self.msg.right_stick_vertical = 0.0
        else:
            self.msg.right_stick_vertical = -axes[self.layout['axis']['RIGHT_STICK_VERTICAL']] # Invert vertical axis

        self.msg.dpad_horizontal = hat_values[self.layout['dpad']['HORIZONTAL']]
        self.msg.dpad_vertical = hat_values[self.layout['dpad']['VERTICAL']]

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
