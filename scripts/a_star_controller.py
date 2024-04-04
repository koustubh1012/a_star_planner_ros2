#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios
from pynput import keyboard
import numpy as np
import heapq as hq
import math
import time

# Define key codes

class AStarControllerNode(Node):

    def __init__(self):
        super().__init__('a_star_controller_node')

        self.declare_parameter('x_pose',0.0)
        self.declare_parameter('y_pose',0.0)
        self.declare_parameter('goal_x',5000.0)
        self.declare_parameter('goal_y',0.0)
        self.declare_parameter('clearance', 50.0)
        self.declare_parameter('rpm1', 30.0)
        self.declare_parameter('rpm2', 40.0)

        self.x_goal = self.get_parameter('goal_x').value
        self.y_goal = self.get_parameter('goal_y').value
        self.x_start = self.get_parameter('x_pose').value
        self.y_start = self.get_parameter('y_pose').value
        self.C = self.get_parameter('clearance').value
        self.rpm1 = self.get_parameter('rpm1').value
        self.rpm2 = self.get_parameter('rpm2').value

        self.get_logger().info('A start controller node initialised')

        self.a_star_solver()











        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def a_star_solver(self):
        obstacle_set = set()             # set to store the obstacle points
        obstacle_list = []               # list to store the obstacle points in order for videp

        c2c_node_grid = [[float('inf')] * 200 for _ in range(600)]       # create a 2D array for storing cost to come
        tc_node_grid = [[float('inf')] * 200 for _ in range(600)]        # create a 2D array for storing cost to come
        closed_set = set()               # set to store the value of visited and closed points                 
        closed_list = []
        visited={}
        self.get_logger().info('My parameter value: %s' % self.x_goal)

        # All units are in cm
        R = 66/20                                                  # Robot wheel radius
        r = 22                                                   # Robot radius
        L = 28.7                                                   # Robot wheel track
        T = self.C + r                                                 # Total clearance

        t_max = 3.5
        t_min = 0.1

        node=(0, 0, 1, [], (self.x_start, self.y_start), 0)
        

        '''
        Loop to define the obstacle points in the map
        '''
        for y in range(200):                                       # loop to define the obstacle points : x
            for x in range(600):                                  # loop to define the obstacle points : y
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

        min_rpm = min(self.rpm1, self.rpm2)

        t = round(((t_max - t_min)*(min_rpm - 75)/(5 - 75)) + t_min, 2)                     # Calculate time step
        self.get_logger().info('Calculated time step : %s' % t)




def main(args=None):
    rclpy.init(args=args)
    node = AStarControllerNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()