#!/bin/bash
sudo apt install -y vim tmux

sudo apt install -y python3-rosdep
sudo apt install -y python3-colcon-common-extensions 

# turtlebot
sudo apt install -y python3-vcstool
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws
wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos
vcs import src < turtlebot3.repos

# nav2
cd ~/turtlebot3_ws/src
git clone https://github.com/ros-planning/navigation2.git --branch foxy-devel

# slam toolbox
git clone -b foxy-devel git@github.com:stevemacenski/slam_toolbox.git

cd ~/turtlebot3_ws

rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy  --os="ubuntu:focal"
colcon build --symlink-install

echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
source ~/.bashrc