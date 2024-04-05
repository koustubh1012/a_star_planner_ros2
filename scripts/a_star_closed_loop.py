#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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
        self.subscription = self.create_subscription(Odometry,'/odom',self.odom_callback, 10)
        self.subscription  # prevent unused variable warning

        self.declare_parameter('x_pose',0.0)
        self.declare_parameter('y_pose',0.0)
        self.declare_parameter('goal_x',5000.0)
        self.declare_parameter('goal_y',0.0)
        self.declare_parameter('clearance', 50.0)
        self.declare_parameter('rpm1', 40.0)
        self.declare_parameter('rpm2', 50.0)

        self.x_goal = int(self.get_parameter('goal_x').value/10) + 50
        self.y_goal = int(self.get_parameter('goal_y').value/10) + 100
        self.x_start = int(self.get_parameter('x_pose').value/10) + 50
        self.y_start = int(self.get_parameter('y_pose').value/10) + 100
        self.C = self.get_parameter('clearance').value/10
        self.rpm1 = self.get_parameter('rpm1').value
        self.rpm2 = self.get_parameter('rpm2').value

    
        self.get_logger().info('A start controller node initialised')

        self.a_star_solver()

        self.i = 1
        self.get_logger().info('Creating Publisher')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.timer_ = self.create_timer(self.t, self.velocity_publisher)  # 1570 ms interval
        self.velocity_msg = Twist()   
        

    def odom_callback(self, msg):
        x = msg.pose.pose.orientation.x  
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        self.yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y*y + z*z))                    # convert quaternion to yaw angle
        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y
        wpt_x = self.waypoints[self.i][0]/100
        wpt_y = self.waypoints[self.i][1]/100
        dist = math.sqrt((robot_x-wpt_x)**2 + (robot_y-wpt_y)**2)
        yaw_req = math.atan2(wpt_y-robot_y, wpt_x-robot_x)
        # if (dist
        
        

    # def velocity_publisher(self):
    #     pass

        
        # self.velocity_msg.linear.x = action[0]/100  # Linear velocity (m/s)
        # self.velocity_msg.angular.z = action[1]  # Angular velocity (rad/s)
        
        # # Publish velocity
        # self.cmd_vel_pub.publish(self.velocity_msg)
        # self.get_logger().info('Publishing velocity: Linear=%.2f, Angular=%.2f' % (self.velocity_msg.linear.x, self.velocity_msg.angular.z))
        


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

        theta_start = 0.0

        node=(0, 0, 1, [], (self.x_start, self.y_start), 0)
        initial_node = (0, 0, 1, [], (self.x_start, self.y_start), theta_start, [])       # create the initial node

        

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

        self.t = round(((t_max - t_min)*(min_rpm - 75)/(5 - 75)) + t_min, 2)                     # Calculate time step
        self.get_logger().info('Calculated time step : %s' % self.t)

        def visited_node(node):
            visited.update({node[2]:node[4]})

        def actionnn(node,rpm1,rpm2):
            ul = 2*math.pi*rpm1/60
            ur = 2*math.pi*rpm2/60
            theta_dot = (R/L)*(ul - ur)
            new_heading = (node[5] + (np.rad2deg(theta_dot)*self.t)) % 360        # get the current heading of the robot
            x_vel = (R/2)*(ur+ul)*np.cos(np.deg2rad(new_heading))
            y_vel = (R/2)*(ur+ul)*np.sin(np.deg2rad(new_heading))
            v_dot = math.sqrt(x_vel**2 + y_vel**2)
            # print("X vel: ", x_vel)
            # print("Y vel: ", y_vel)
            x = node[4][0] + x_vel*self.t # calculate the new x coordinate
            y = node[4][1] + y_vel*self.t # calculate the new y coordinate
            x = round(x) 
            y = round(y)
            c2c = node[1] + math.sqrt((x_vel*self.t)**2 + (y_vel*self.t)**2)                                    # calculate the cost to come
            c2g = math.sqrt((self.y_goal-y)**2 + (self.x_goal-x)**2)   # calculate the cost to goal
            tc = c2c + c2g                                   # calculate the total cost
            return (x,y),new_heading,tc,c2c                # return the new node's coordinates, heading, total cost and cost to come

        action_lists=[(0,self.rpm1),(self.rpm1,0),(self.rpm1,self.rpm1),(self.rpm1,self.rpm2),
                      (self.rpm2,self.rpm1),(0,self.rpm2),(self.rpm2,0),(self.rpm2,self.rpm2)]
        
        new_index = 1         
        open_list = []
        hq.heappush(open_list,initial_node)        # Push initial node to the list
        hq.heapify(open_list)                      # covers list to heapq data type

        while(open_list):
            node = hq.heappop(open_list)       # pop the node with lowest cost to come
            closed_list.append(node[4])            # add the node coordinates to closed set
            closed_set.add(node[4])
            visited_node(node)                 # add the node to the visited list
            index = node[2]                    # store the index of the current node
            parent_index = node[3]             # store the parent index list of current node

            node_dist = math.sqrt((node[4][0]-self.x_goal)**2 + (node[4][1]-self.y_goal)**2)     # calculate the distance between the current node and goal node
            if node_dist < 5:    # if the node is goal position, exit the loop
                print("Goal reached")
                break

            for action_set in action_lists:
                point, new_heading, tc, c2c= actionnn(node,action_set[0],action_set[1])
                if point not in obstacle_set and point not in closed_set and 0<=point[0]<600 and 0<=point[1]<200:           # check if the new node is in the obstacle set or visited list
                    x = int(point[0])                                                    # get the x coordinate of the new node
                    y = int(point[1])                                                    # get the y coordinate of the new node
                    try:
                        if tc<tc_node_grid[x][y]:                                       # check if the new cost to come is less than original cost to come
                            new_parent_index = parent_index.copy()                      # copy the parent index list of the current node
                            new_parent_index.append(index)                              # Append the current node's index to the new node's parent index list
                            new_index+=1                                                # increment the index
                            tc_node_grid[x][y] = tc                                     # Update the new total cost
                            c2c_node_grid[x][y] = c2c                                   # Update the new cost to come
                            new_node = (tc, c2c, new_index, new_parent_index, (x,y), new_heading) # create the new node
                            hq.heappush(open_list, new_node)                            # push the new node to the open list
                    except:
                        pass

        print(len(node[3]))
        path = node[3]
        self.waypoints = []
        print("Actual goal reached :",(node[4][0]-50)*10, (node[4][1]-100)*10)
        for index in path:                                                        # loop to mark the path
            coord=visited[index]                                                  # get the coordinates of the node
            x = (coord[0]-50)/100
            y = (coord[1]-100)/100
            self.waypoints.append((x, y))
        # self.waypoints.append
        print("Waypoints: ",self.waypoints)

        




def main(args=None):
    
    rclpy.init(args=args) # Initialize the ROS client library
    node = AStarControllerNode() # Create an instance of the AStarControllerNode
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().error("KeyboardInterrupt received!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()