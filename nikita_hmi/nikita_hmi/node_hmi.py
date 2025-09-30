#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__copyright__ = "Copyright (C) 2025 Christian Stein"
__license__ = "MIT"

import subprocess
import time
import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from nikita_interfaces.msg import MovementRequest
from nikita_interfaces.msg import ServoStatus

from PIL import Image, ImageDraw, ImageFont

import board
from digitalio import DigitalInOut
import adafruit_ssd1306
import adafruit_bno055

# TODO ideas for display_text:
# - hottest servo temperature plus servo name
# - supply voltage of the servo (servo name not relevant)
# - servo relay status
# - listening active
# - battery status


class NodeHmi(Node):
    NUMBER_OF_LINES = 6

    def __init__(self):
        super().__init__('node_hmi')

        self.lidar_distance = 99.9
        self.max_temperature = 20.0
        self.servo_name_max_temperature = "HEAD_YAW" # 21 characters are possible, e.g. LEG_RIGHT_FRONT_FEMUR
        self.servo_min_voltage = 12.0
        self.movement_request = 'unknown'
        self.is_ip_address_identified = False
        self.timeout_identifying_ip_address = 60
        self.display_text = ["" for x in range(self.NUMBER_OF_LINES)]

        # Servo Relay
        self.relay_pin = DigitalInOut(board.D23) # PIN16, GPIO_23
        self.relay_pin.switch_to_output()
        self.relay_pin.value = False # turn relay off

        self.i2c = board.I2C()
        self.display = adafruit_ssd1306.SSD1306_I2C(128, 64, self.i2c)
        self.display.fill(0)
        self.display.show()
        self.image = Image.new("1", (self.display.width, self.display.height))
        self.draw = ImageDraw.Draw(self.image)
        self.font = ImageFont.truetype("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf", 12)

        self.__bno055 = adafruit_bno055.BNO055_I2C(self.i2c)
        time.sleep(1)

        # turn relay on
        self.relay_pin.value = True

        # TODO change display_text to a topic of interest, battery, etc.
        self.create_subscription(Float32, 'lidar', self.callback_lidar, 10)
        self.create_subscription(Bool, 'request_servo_relay', self.callback_servo_relay, 10)
        self.create_subscription(Bool, 'request_system_shutdown', self.callback_system_shutdown, 10)
        self.create_subscription(MovementRequest, 'movement_request', self.callback_movement_request, 10)
        self.create_subscription(ServoStatus, 'servo_status', self.callback_servo_status, 10)
        self.pub_gravity = self.create_publisher(Vector3, 'gravity', 10)

        # display IP address
        text = "IP: " + self.get_ip_address()
        # TODO TEST easier like this self.font_height = 10 + 2
        (self.font_width, self.font_height) = self.font.getmask(text).size
        self.font_height += 3
        # First line always shows the IP address
        self.update_display(text, 0)
        self.display_gravity()
        self.display_lidar()
        self.display_movement_request()

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # TODO: for faster publishing the gravity topic an additional timer is needed.

    def update_display(self, text, line_number=0):
        # first overwrite the old text with the same text in black
        self.draw.text((0, self.font_height * line_number), self.display_text[line_number], font=self.font, fill=0)
        # then write the new text in white
        self.draw.text((0, self.font_height * line_number), text, font=self.font, fill=255)
        self.display.image(self.image)
        self.display.show()
        # update the text in the self.display_text array
        self.display_text[line_number] = text

    def get_ip_address(self):
        ip_address = "0.0.0.0"
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8",80))
            ip_address = s.getsockname()[0]
            s.close()
            self.is_ip_address_identified = True
        except Exception as e:
            self.get_logger().warning('Could not get IP address, error: %s' % e)
            ip_address = "0.0.0.0"  # fallback

        self.get_logger().info('IP address: %s' % ip_address)
        return ip_address

    def display_gravity(self):
        (x,y,z) = self.__bno055.gravity
        text = f"g: {z:.2f}m/s^2"
        # self.update_display(text, 1)   # not enough space for this information
        return (x,y,z)

    def display_movement_request(self):
        text = self.movement_request
        self.update_display(text, 1)

    def display_lidar(self):
        text = f"d: {self.lidar_distance::>5.2f}m"   # right justified, 5 characters wide, 2 decimal places
        self.update_display(text, 2)

    def display_servo_status(self):
        text = f"{self.servo_name_max_temperature}"
        self.update_display(text, 3)
        text = f"{self.servo_min_voltage:.1f}V | {self.max_temperature:.1f}°C"
        self.update_display(text, 4)

    def timer_callback(self):
        if not self.is_ip_address_identified and self.timeout_identifying_ip_address > 0:
            self.timeout_identifying_ip_address -= 1
            ip_address = self.get_ip_address()

            if self.is_ip_address_identified:
                text = "IP: " + ip_address
                self.update_display(text, 0)

        (x,y,z) = self.display_gravity()
        self.display_lidar()

        # publish gravity
        msg = Vector3()
        msg.x = x
        msg.y = y
        msg.z = z
        self.pub_gravity.publish(msg)

    def callback_lidar(self, msg):
        self.lidar_distance = msg.data

    def callback_movement_request(self, msg):
        self.movement_request = msg.name.lower()
        self.display_movement_request()

    def callback_servo_status(self, msg):
        self.max_temperature = msg.max_temperature
        self.servo_name_max_temperature = msg.servo_max_temperature.lower()
        self.servo_min_voltage = msg.min_voltage
        self.display_servo_status()

    def callback_servo_relay(self, msg):
        self.get_logger().info('callback_servo_relay: %s' % msg.data)
        if msg.data:
            self.relay_pin.value = True
        else:
            self.relay_pin.value = False
    
    def callback_system_shutdown(self, msg):
        self.get_logger().info('callback_system_shutdown: %s' % msg.data)
        if msg.data:
            self.shutdown_callback()
            time.sleep(1)
            self.get_logger().info('3')
            time.sleep(1)
            self.get_logger().info('2')
            time.sleep(1)
            self.get_logger().info('1')
            time.sleep(1)
            subprocess.call("sudo shutdown -h now", shell=True)  # sudo visudo ->  nikita ALL=(ALL) NOPASSWD: /sbin/shutdown

    def shutdown_callback(self):
        self.get_logger().info("shutdown_callback")
        self.relay_pin.value = False
        self.movement_request = "!!SHUTDOWN!!"
        self.display_movement_request()


def main(args=None):
    rclpy.init(args=args)
    node_hmi = NodeHmi()

    try:
        rclpy.spin(node_hmi)  # Spin the node to process callbacks
    except KeyboardInterrupt:
        node_hmi.get_logger().info("Node interrupted by user.")
    finally:
        node_hmi.shutdown_callback()
        node_hmi.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
