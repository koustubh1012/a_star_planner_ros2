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
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_project3'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')


    declare_x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0',description='Initial x pose')
    declare_y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0',description='Initial y pose')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    a_star_controller_node = Node(
        package='turtlebot3_project3',
        executable='a_star_controller.py',
        name='a_star_controller',
        output='screen',
        parameters=[{'x_pose': x_pose, 'y_pose': y_pose, 'use_sim_time': use_sim_time}]
    )


    ld.add_action(declare_x_pose_arg)
    ld.add_action(declare_y_pose_arg)
    ld.add_action(a_star_controller_node)

    return ld
