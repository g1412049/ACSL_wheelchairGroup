#!/bin/bash
stty -echo
read -p "Password: " password
# ROS melodic の Ubuntu へのインストール
echo "$password" | sudo -S sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
echo "$password" | sudo -S apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
echo "$password" | sudo -S apt update
echo "$password" | sudo -S apt install ros-melodic-desktop-full
echo "$password" | sudo -S rosdep init
rosdep update
echo "# Set ROS melodic" >> ~/.bashrc
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "# Set ROS Network" >> ~/.bashrc
echo "export ROS_HOSTNAME=127.0.0.1" >> ~/.bashrc
echo 'export ROS_MASTER_URI=http://${ROS_HOSTNAME}:11311' >> ~/.bashrc
echo "" >> ~/.bashrc
echo "# Set ROS alias command" >> ~/.bashrc
echo "alias cw='cd ~/catkin_ws'" >> ~/.bashrc
echo "alias cs='cd ~/catkin_ws/src'" >> ~/.bashrc
echo "alias cm='cd ~/catkin_ws && catkin_make'" >> ~/.bashrc
source ~/.bashrc
echo "$password" | sudo -S apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
# Gazeboのros連携するプラグインのインストール
echo "$password" | sudo -S apt-get install -y ros-melodic-gazebo-ros-control
echo "$password" | sudo -S apt-get install -y ros-melodic-ros-control ros-melodic-ros-controllers
