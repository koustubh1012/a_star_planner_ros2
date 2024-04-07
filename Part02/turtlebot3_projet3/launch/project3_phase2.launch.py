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
    C = 50
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

    valid_start = False                                                             # flag to check if the start point is valid

    while not valid_start:                                                          # loop to check if the start point is valid
            start = input("Enter the start coordinates and orientation as (x, y, theta): ")                         # get the start coordinate and orientaion from the user
            [x_start, y_start, theta_start] = [int(i) for i in start.split()]
            x_start = int(x_start/10) + 50
            y_start = int(y_start/10) + 100
            if (x_start, y_start) in obstacle_set:                                  # check if the start point is in the obstacle set
                print("Invalid coordinates, Enter again")                         # print error message
            else:
                initial_node = (0, 0, 1, [], (x_start, y_start), theta_start)       # create the initial node
                valid_start = True                                                  # set the flag to true

    valid_goal = False                                                              # flag to check if the goal point is valid

    while not valid_goal:
            goal = input("Enter the goal coordinates as (x, y): ")                         # get the start coordinate and orientaion from the user
            [x_goal, y_goal] = [int(i) for i in goal.split()]
            x_goal = int(x_goal/10) + 50
            y_goal = int(y_goal/10) + 100
            if (x_goal, y_goal) in obstacle_set:                                   # check if the goal point is in the obstacle set
                print("Invalid coordinates, Enter again")                        # print error message
            else:
                goal = (x_goal, y_goal)                                            # create the goal node
                valid_goal = True                                                  # set the flag to true


    ld = LaunchDescription()
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_project3'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    goal_x = LaunchConfiguration('goal_x', default='5000.0')
    goal_y = LaunchConfiguration('goal_y', default='0.0')

    a_star_controller_node = Node(
        package='turtlebot3_project3',
        executable='a_star_closed_loop.py',
        name='a_star_controller_node',
        output='screen',
        parameters=[{'x_pose': x_pose}, 
                    {'y_pose': y_pose}, 
                    {'goal_x': goal_x},
                    {'goal_y': goal_y}]
    )

    ld.add_action(a_star_controller_node)

    return ld

C = 50
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



# while True:
#     try:  
#         x_start = input("Enter the x coordinate of the start point: ")
#         x_pose = int(x_start) / 10 + 50
#         y_start = input("Enter the y coordinate of the start point: ")
#         y_pose = int(y_start) / 10 + 100
#         x_goal = input("Enter the x coordinate of the goal point: ")
#         x_goal = int(x_goal) / 10 + 50
#         y_goal = input("Enter the y coordinate of the goal point: ")
#         y_goal = int(y_goal) / 10 + 100
#     except ValueError:  
#         print("Invalid input: Please enter numeric values for coordinates.")
#         continue  
#     if x_start or y_start  in obstacle_set:
#         print("Invalid coordinates: Obstacle detected. Please enter valid values.")
#     elif x_goal or y_goal in obstacle_set:
#         print("Invalid coordinates: Obstacle detected. Please enter valid values.")
#     else:
#         break  
