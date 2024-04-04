#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_project3'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    a_start_controller_node = Node(
            package='turtlebot3_project3',
            # namespace='turtlebot3',
            executable='a_start_controller.py',
            name='a_start_controller'
        )


    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(a_start_controller_node)

    return ld
