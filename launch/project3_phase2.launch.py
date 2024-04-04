#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration                                                     
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    ld = LaunchDescription()
    cmd_line_parameter = DeclareLaunchArgument('cmd_line_parameter', default_value='default_value', description='Description of the command line parameter')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    a_star_controller_node = Node(
        package='project3_phase2',
        executable='a_star_controller',
        name='a_star_controller',
        output='screen',
        parameters=[{'x_pose': x_pose, 'y_pose': y_pose}]
    )

    ld.add_action(cmd_line_parameter)

    return ld
