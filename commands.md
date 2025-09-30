## service
```
sudo systemctl start autostart_ros2
sudo systemctl status autostart_ros2
```

## install dependencies
```
PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install --from-paths ~/Workspace/colcon_nikita --ignore-src -r -y

python3 -m pip install lewansoul-lx16a --break-system-packages
python3 -m pip install lewansoul-lx16a-terminal --break-system-packages

git submodule update --init
```

## build
```
colcon build --symlink-install
colcon build --packages-up-to nikita_servo
```

```
source install/local_setup.bash
```

## log topics
```
ros2 topic list
ros2 topic echo --csv /servos_status
ros2 topic echo --once /servos_status
```

## send requests
```
ros2 topic pub --once /request_listening std_msgs/msg/Bool data:\ true

ros2 topic pub --once /speech_recognition_online std_msgs/msg/String data:\ "fahre im viereck"

ros2 topic pub --once /speech_recognition_online std_msgs/msg/String "{data: 'steh auf'}"

ros2 topic pub --once /speech_recognition_online std_msgs/msg/String "{data: 'leg dich hin'}"

ros2 topic pub --once /speech_recognition_online std_msgs/msg/String "{data: 'mach dich transport bereit'}"

ros2 topic pub --once /speech_recognition_online std_msgs/msg/String data:\ "teste beine"

ros2 topic pub --once /request_talking std_msgs/msg/String "{data: 'mach dich transport bereit'}"

ros2 topic pub --once /request_music std_msgs/msg/String "{data: 'musicfox_hot_dogs_for_breakfast.mp3'}"

ros2 topic pub --once /joystick_request nikita_interfaces/msg/JoystickRequest "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
button_a: false
button_b: false
button_x: false
button_y: false
button_l1: false
button_l2: false
button_r1: false
button_r2: false
button_select: false
button_start: false
dpad_vertical: 0
dpad_horizontal: 0
left_stick_vertical: 0.10
left_stick_horizontal: 0.0
right_stick_horizontal: 0.0
right_stick_vertical: 0.0"
---

ros2 topic pub --once /movement_request nikita_interfaces/msg/MovementRequest "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
type: 1
duration_ms: 2000
name: 'manual'
velocity:
  linear:
    x: 0.00
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
body:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
"
---

ros2 topic pub --once /servo_status nikita_interfaces/msg/ServoStatus "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
servo_error_code: 'LEG_LEFT_FRONT_COXA'
error_code: 0
servo_max_temperature: 'LEG_LEFT_FRONT_COXA'
max_temperature: 40.0
servo_max_voltage: 'LEG_LEFT_FRONT_COXA'
max_voltage: 12.0
servo_min_voltage: 'LEG_LEFT_FRONT_COXA'
min_voltage: 12.0
"
---

ros2 topic pub --once /servo_request nikita_interfaces/msg/ServoRequest "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
target_angles:
- name: 'HEAD_YAW'
  angle_deg: 0.0
- name: 'HEAD_PITCH'
  angle_deg: 0.0
- name: 'LEG_LEFT_FRONT_COXA'
  angle_deg: 0.0
- name: 'LEG_LEFT_FRONT_FEMUR'
  angle_deg: 20.0
- name: 'LEG_LEFT_FRONT_TIBIA'
  angle_deg: 20.0
- name: 'LEG_LEFT_MID_COXA'
  angle_deg: 0.0
- name: 'LEG_LEFT_MID_FEMUR'
  angle_deg: 0.0
- name: 'LEG_LEFT_MID_TIBIA'
  angle_deg: 0.0
- name: 'LEG_LEFT_BACK_COXA'
  angle_deg: 0.0
- name: 'LEG_LEFT_BACK_FEMUR'
  angle_deg: 0.0
- name: 'LEG_LEFT_BACK_TIBIA'
  angle_deg: 0.0
- name: 'LEG_RIGHT_FRONT_COXA'
  angle_deg: 0.0
- name: 'LEG_RIGHT_FRONT_FEMUR'
  angle_deg: 0.0
- name: 'LEG_RIGHT_FRONT_TIBIA'
  angle_deg: 0.0
- name: 'LEG_RIGHT_MID_COXA'
  angle_deg: 0.0
- name: 'LEG_RIGHT_MID_FEMUR'
  angle_deg: 0.0
- name: 'LEG_RIGHT_MID_TIBIA'
  angle_deg: 0.0
- name: 'LEG_RIGHT_BACK_COXA'
  angle_deg: 0.0
- name: 'LEG_RIGHT_BACK_FEMUR'
  angle_deg: 0.0
- name: 'LEG_RIGHT_BACK_TIBIA'
  angle_deg: 0.0
time_to_reach_target_angles_ms: 3000"
```

```
ros2 topic pub --once /single_servo_request nikita_interfaces/msg/ServoAngle "
  name: LEG_RIGHT_BACK_COXA 
  angle_deg: 0.0"
```

## launch robot
```
source install/local_setup.bash
ros2 launch nikita_bringup target_launch.py
ros2 launch nikita_bringup test_launch.py

ros2 launch nikita_brain brain_launch.py
ros2 launch nikita_communication communication_launch.py
ros2 launch nikita_movement movement_launch.py
ros2 launch nikita_servo servo_launch.py
ros2 launch nikita_teleop teleop_launch.py
ros2 launch nikita_lidar lidar_launch.yaml

#ros2 launch nikita_servo_controller servo_controller_launch.py


```

## remote computer

