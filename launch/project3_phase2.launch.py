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

C = int(input("Enter the clearance value: "))

rpm1 = int(input("Enter the RPM1 value: "))
rpm2 = int(input("Enter the RPM2 value: "))

C = C/10
R = 66/20
r = 22
L = 28.7
T = C + r  

t_max = 3.5
t_min = 0.1
theta_start = 0.0

obstacle_set = set()  # set to store the obstacle points
obstacle_list = []  # list to store the obstacle points in order for video

c2c_node_grid = [[float('inf')] * 200 for _ in range(600)]  # create a 2D array for storing cost to come
tc_node_grid = [[float('inf')] * 200 for _ in range(600)]  # create a 2D array for storing cost to come

for y in range(200):                                       # loop to define the obstacle points : x
    for x in range(600):                                  # loop to define the obstacle points : y
        # canvas[y,x] = [255,255,255]                        # mark the points in the frame with white color
        if (0<=y<=T):                                      # points in the bottom boundary
            obstacle_set.add((x,y))                        # add the points to the obstacle set
            obstacle_list.append((x,y))                    # add the points to the obstacle list
            c2c_node_grid[x][y] = -1                       # mark the points in the cost to come grid with -1
            tc_node_grid[x][y] = -1                        # mark the points in the total cost grid with -1
        elif (0<=x<=T):                                    # points in the left boundary
            obstacle_set.add((x,y))                        # add the points to the obstacle set
            obstacle_list.append((x,y))                    # add the points to the obstacle list
            c2c_node_grid[x][y] = -1                       # mark the points in the cost to come grid with -1
            tc_node_grid[x][y] = -1                        # mark the points in the total cost grid with -1
        elif (200-T<=y<200):                               # points in the top boundary
            obstacle_set.add((x,y))                        # add the points to the obstacle set
            obstacle_list.append((x,y))                    # add the points to the obstacle list
            c2c_node_grid[x][y] = -1                       # mark the points in the cost to come grid with -1
            tc_node_grid[x][y] = -1                        # mark the points in the total cost grid with -1
        elif (600-T<=x<600):                             # points in the right boundary
            obstacle_set.add((x,y))                        # add the points to the obstacle set
            obstacle_list.append((x,y))                    # add the points to the obstacle list
            c2c_node_grid[x][y] = -1                       # mark the points in the cost to come grid with -1
            tc_node_grid[x][y] = -1                        # mark the points in the total cost grid with -1
        
        elif (150-T<=x<=175+T) and (100-T<=y<=200):    # points in first rectangle
            obstacle_set.add((x,y))                        # add the points to the obstacle set
            obstacle_list.append((x,y))                    # add the points to the obstacle list
            c2c_node_grid[x][y] = -1                       # mark the points in the cost to come grid with -1
            tc_node_grid[x][y] = -1                        # mark the points in the total cost grid with -1

        elif (250-T<=x<=275+T) and (0<=y<=100+T):       # points in second rectangle
            obstacle_set.add((x,y))                        # add the points to the obstacle set
            obstacle_list.append((x,y))                    # add the points to the obstacle list
            c2c_node_grid[x][y] = -1                       # mark the points in the cost to come grid with -1
            tc_node_grid[x][y] = -1                        # mark the points in the total cost grid with -1
         
        # Points in the Circle shaped obstacle
        elif ((x-420)**2 + (y-120)**2 <= (60+T)**2):      # points in the first rectangle of Concave shaped obstacle
            obstacle_set.add((x,y))                        # add the points to the obstacle set
            obstacle_list.append((x,y))                    # add the points to the obstacle list
            c2c_node_grid[x][y] = -1                       # mark the points in the cost to come grid with -1
            tc_node_grid[x][y] = -1                        # mark the points in the total cost grid with -1

while True:
    try:  
        x_start = input("Enter the x coordinate of the start point: ")
        x_pose = int(x_start) // 10 + 50
        y_start = input("Enter the y coordinate of the start point: ")
        y_pose = int(y_start) // 10 + 100
        x_goal = input("Enter the x coordinate of the goal point: ")
        x_goal = int(x_goal) // 10 + 50
        y_goal = input("Enter the y coordinate of the goal point: ")
        y_goal = int(y_goal) // 10 + 100
    except ValueError:  
        print("Invalid input: Please enter numeric values for coordinates.")
        continue  
    if x_start or y_start  in obstacle_set:
        print("Invalid coordinates: Obstacle detected. Please enter valid values.")
    elif x_goal or y_goal in obstacle_set:
        print("Invalid coordinates: Obstacle detected. Please enter valid values.")
    else:
        break  
