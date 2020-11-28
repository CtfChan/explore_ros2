# explore_ros2
WIP based on https://github.com/hrnr/m-explore

# Prereq

# Setup



# Personal Notes

colcon-test --package-select explore_ros2

unit test executable is in `workspace/build/explore_ros2/test`



ros2 launch nav2_bringup just_turtlebot.py
ros2 launch nav2_bringup slam_launch.py
ros2 launch nav2_bringup navigation_launch.py
ros2 run explore_ros2 explorer __log_level:=debug



https://github.com/ros2/examples/tree/master/rclcpp/actions/minimal_action_client


--------------
Action to control
/navigate_to_pose

---------------
NavigateToPose.action

#goal definition
geometry_msgs/PoseStamped pose
string behavior_tree
---
#result definition
std_msgs/Empty result
---
geometry_msgs/PoseStamped current_pose
builtin_interfaces/Duration navigation_time
int16 number_of_recoveries
float32 distance_remaining
