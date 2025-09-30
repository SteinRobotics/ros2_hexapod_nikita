# ROS2 node for Hiwonder Bus Servos

## Bugfix for serial communication not working
https://askubuntu.com/questions/1066150/ubuntu-usb-to-serial-device-connection-problem
sudo apt remove brltty  # remove brltty

ls -la /dev/ttyUSB0
crw-rw---- 1 root dialout 188, 0 Sep 27 10:39 /dev/ttyUSB0
sudo adduser christian dialout


## Servo Layout

```
           020
           010
            |
036-026-016-|-011-021-031
            |
035-025-015-|-012-022-032
            |
034-024-014-|-013-023-033
```

## Code
based on:
Hiwonder Bus Servo CCommunication Protocol.pdf
https://github.com/Duffmann/LSServo

tool for inital servo setup:
https://github.com/maximkulkin/lewansoul-lx16a


## Commands
source install/local_setup.bash
<!-- ros2 run nikita_servo_controller node_servo_controller --ros-args -p device_name:="/dev/ttyUSB0" -->
ros2 run nikita_servo_controller node_servo_controller --ros-args -p device_name:="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"


dmesg:
new full-speed USB device number 8 using xhci_hcd
New USB device found, idVendor=1a86, idProduct=7523, bcdDevice=81.34
New USB device strings: Mfr=0, Product=2, SerialNumber=0
Product: USB Serial