# explore_ros2
This package is a ROS2 port of the following repo: https://github.com/hrnr/m-explore.

[![](http://img.youtube.com/vi/D_3IMG56U80/0.jpg)](http://www.youtube.com/watch?v=D_3IMG56U80 "")

# Prerequisite
- ROS2 Foxy (I have only tested it on Foxy)
- Navigation2 package
- Turtlebot3 package

Please follow the Navigation2 [Getting Started](https://navigation.ros.org/getting_started/index.html) page to install the required packages and setup the required environment variables.

# Building
```
$ cd ~/my_ws/src
$ git clone https://github.com/CtfChan/explore_ros2.git
$ cd ~/my_ws
$ colcon build --packages-select explore_ros2 --symlink-install
$ . install/local_setup.bash
```

# Running Demo
Launching too many nodes at the same time caused my laptop to crash. Launch the following commands in four separate panes using the order below. 

```
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 
$ ros2 launch nav2_bringup slam_launch.py
$ ros2 launch nav2_bringup navigation_launch.py
$ ros2 launch explore_ros2 explore_demo.py
```

# Running Unit Test
Work in progress...

colcon-test --package-select explore_ros2

unit test executable is in `workspace/build/explore_ros2/test`
