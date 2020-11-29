from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    explore_ros2_dir = get_package_share_directory('explore_ros2')

    rviz_config_file = os.path.join(explore_ros2_dir, 'rviz', 'explore.rviz')

    return LaunchDescription([
        Node(
            package='explore_ros2',
            executable='explorer',
            name='explorer_node',
            parameters=[
                {"potential_scale": 1e-3},
                {"gain_scale": 1.0},
                {"min_frontier_size": 0.5},
                {"orientation_scale": 0.0},
                {"progress_timeout": 30},
                {"visualize": True}
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file])
    ])