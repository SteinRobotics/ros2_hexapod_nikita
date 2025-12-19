sudo apt update
sudo apt upgrade
sudo reboot
sudo shutdown -h now
mkdir Workspace

____
## install Workspace
### install ROS2
```
mkdir colcon_nikita
cd colcon_nikita/
mkdir src
cd src/
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt install ros-dev-tools
sudo apt update
sudo apt upgrade
sudo apt install ros-kilted-ros-base
```

### install nikita 
```
git clone https://github.com/SteinRobotics/ros2_hexapod_nikita.git
cd ros2_hexapod_nikita/
git checkout develop 
git pull
sudo rosdep init
rosdep update
sudo apt install python3-pip
PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install --from-paths ~/Workspace/colcon_nikita --ignore-src -r -y
git clone https://github.com/garmin/LIDARLite_RaspberryPi_Library.git
sudo apt install libmagicenum-dev
nano ~/Workspace/colcon_nikita/src/ros2_hexapod_nikita/nikita_lidar/src/LIDARLite_RaspberryPi_Library/include/lidarlite_v3.h 
colcon build --symlink-install
source /opt/ros/kilted/setup.bash
source ~/.bashrc
```

### install dependencies
```
python3 -m pip install --break-system-packages vosk pyttsx3 google-api-python-client google-cloud-speech oauth2client sounddevice SpeechRecognition pyyaml
sudo apt install -y portaudio19-dev python3-pyaudio
sudo pip3 install adafruit-circuitpython-ssd1306 --break-system-packages
sudo pip3 install adafruit-circuitpython-bno055 --break-system-packages
```

___
## Sound settings:
```
wpctl get-volume @DEFAULT_AUDIO_SINK@
wpctl set-volume @DEFAULT_AUDIO_SINK@ 0.90
wpctl status
```
___  
