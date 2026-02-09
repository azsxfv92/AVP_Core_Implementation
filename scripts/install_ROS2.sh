#!/bin/bash
set -e

echo "[1/5] Locale setting (UTF-8)"
sudo apt update && sudo apt install locales -y
sudo locale-gen ko_KR ko_KR.UTF-8
sudo update-locale LC_ALL=ko_KR.UTF-8 LANG=ko_KR.UTF-8
export LANG=ko_KR.UTF-8

echo "[2/5] Register ROS 2 Humble repository"
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "[3/5] ROS 2 Humble Desktop installation (Desktop-Full)"
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop -y

echo "[4/5] Development tool installation (colcon, rosdep, vcstool)"
sudo apt install python3-colcon-common-extensions python3-rosdep python3-argcomplete python3-vcstool -y

echo "[5/5] Set environment variable and rosdep"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# Register source in .bashrc
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
fi

echo "Complete installing ROS 2 Humble."