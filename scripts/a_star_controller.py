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
        self.declare_parameter('goal_x',0.0)
        self.declare_parameter('goal_y',0.0)
        self.declare_parameter('clearance', 50.0)

        x_goal = self.get_parameter('goal_x').value
        y_goal = self.get_parameter('goal_y').value
        x_start = self.get_parameter('x_pose').value
        y_start = self.get_parameter('y_pose').value
        self.get_logger().info('My parameter value: %s' % x_goal)
        node=(0, 0, 1, [], (x_start, y_start), 0)
        
        self.C = 5
        self.R = 66/20                                                  # Robot wheel radius
        self.r = 22                                                   # Robot radius
        self.L = 28.7                                                   # Robot wheel track
        self.T = self.C + self.r                                                 # Total clearance
        self.t = 1

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def a_star_solver(self):
        obstacle_set = set()             # set to store the obstacle points
        obstacle_list = []               # list to store the obstacle points in order for videp

        c2c_node_grid = [[float('inf')] * 200 for _ in range(600)]       # create a 2D array for storing cost to come
        tc_node_grid = [[float('inf')] * 200 for _ in range(600)]        # create a 2D array for storing cost to come
        closed_set = set()               # set to store the value of visited and closed points                 
        closed_list = []
        visited={}

        # All units are in cm
        



def main(args=None):
    rclpy.init(args=args)
    node = AStarControllerNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()