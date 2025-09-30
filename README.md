
# Nikita Hexapod Robot (Maker Project)

**⚠️ WARNING: This repository is under heavy development! Breaking changes, incomplete features, and experimental code are expected. Use at your own risk. Contributions, feedback, and ideas are welcome.**

Nikita is an open-source, modular hexapod robot platform for makers, tinkerers, and robotics enthusiasts. The project is built around ROS2 and aims to be a flexible playground for learning, hacking, and experimenting with robotics and human-machine interaction. 

## Features & Goals
- **Maker Focus**: Designed for hands-on experimentation, learning, and creative robotics projects.
- **ROS2 Native**: Modern robotics workflows and easy integration with other ROS2 packages.
- **Modular Architecture**: Movement, communication, HMI, teleoperation, servo control, and more
- **Speech Recognition & Synthesis**: Online/offline STT and TTS with caching.
- **Flexible Gait & Kinematics**: Advanced gait controller and kinematics for smooth, stable walking and body pose control.
- **Teleoperation**: Joystick and remote control support.
- **Extensive Launch & Config**: Launch files and configuration options for different hardware and scenarios.
- **Diagnostics & Services**: Systemd integration and ROS2 topics for monitoring and diagnostics.


## Project Structure
- `nikita_brain/`         — High-level behavior, action planning, and coordination
- `nikita_movement/`      — Gait, kinematics, and movement primitives
- `nikita_servo/`         — Servo abstraction and low-level control
- `nikita_servo_controller/` — Advanced servo controller node
- `nikita_communication/` — Speech recognition, TTS, and chatbot integration
- `nikita_hmi/`           — Human-machine interface (GUI, feedback)
- `nikita_teleop/`        — Teleoperation (joystick, remote)
- `nikita_lidar/`         — LIDAR sensor integration
- `nikita_interfaces/`    — Custom ROS2 message and service definitions
- `nikita_bringup/`       — Launch and bringup scripts
- `nikita_doc/`           — Documentation, diagrams, and hardware info

## Quick Start (for Makers)
1. **Install Dependencies**
   ```bash
   PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install --from-paths ~/Workspace/colcon_nikita --ignore-src -r -y
   git submodule update --init
   ```
2. **Build the Workspace**
   ```bash
   colcon build --symlink-install
   source install/local_setup.bash
   ```
3. **Launch the Robot**
   ```bash
   ros2 launch nikita_bringup target_launch.py
   # or for testing
   ros2 launch nikita_bringup test_launch.py
   ```
4. **Launch Individual Components**
   ```bash
   ros2 launch nikita_brain brain_launch.py
   ros2 launch nikita_communication communication_launch.py
   ros2 launch nikita_movement movement_launch.py
   ros2 launch nikita_servo servo_launch.py
   ros2 launch nikita_teleop teleop_launch.py
   ros2 launch nikita_lidar lidar_launch.yaml
   ```
5. **Interact & Hack**
   - Send movement commands:
     ```bash
     ros2 topic pub --once /movement_request nikita_interfaces/msg/MovementRequest "..."
     ```
   - Monitor topics:
     ```bash
     ros2 topic list
     ros2 topic echo /servos_status
     ```
   - Speech commands:
     ```bash
     ros2 topic pub --once /speech_recognition_online std_msgs/msg/String "{data: 'steh auf'}"
     ```
   - Joystick/teleop:
     ```bash
     ros2 topic pub --once /joystick_request nikita_interfaces/msg/JoystickRequest "..."
     ```


## Systemd Service (Optional)
To auto-start ROS2 on boot:
```bash
sudo systemctl start autostart_ros2
sudo systemctl status autostart_ros2
```


## Documentation
- See `nikita_doc/` for hardware pinouts, protocol docs, and setup guides.
- Diagrams and images are provided for wiring and architecture overview.


## Contributing
This is a maker project—experimentation, hacking, and learning are encouraged! Contributions, bug reports, and feature requests are welcome. Please open issues or pull requests for improvements.


## License
Copyright (c) 2021-2025 Christian Stein

---

