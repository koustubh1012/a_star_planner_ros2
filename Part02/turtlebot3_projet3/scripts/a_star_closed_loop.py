# ENPM 661: Planning for Robotics
# Project 3 - Phase 2
# authors: Keyur Borad, Aryan Mishra, FNU Koustubh
# This script is used to implement the A* algorithm for the Turtlebot3 robot to navigate in a known environment
#github link:https://github.com/koustubh1012/a_star_planner_ros2

#!/usr/bin/env python3
 
import rclpy                                                                   # Import the ROS client library for Python
from rclpy.node import Node                                                    # Import the Node class from the rclpy library
from geometry_msgs.msg import Twist                                            # Import the Twist message from the geometry_msgs package
from nav_msgs.msg import Odometry                                              # Import the Odometry message from the nav_msgs package
from pynput import keyboard                                                    # Import the keyboard module from the pynput library
import numpy as np                                                             # Import the numpy library
import heapq as hq                                                             # Import the heapq library
import math                                                                    # Import the math library

# Define key codes

class AStarControllerNode(Node):
    """
    AStarControllerNode class is a subclass of the Node class

    Args:
        Node (Class): The Node class from the rclpy library
        subscription (Subscription): The subscription object to subscribe to the /odom topic
        cmd_vel_pub (Publisher): The publisher object to publish to the /cmd_vel topic
        velocity_msg (Twist): The Twist message object to publish the velocity commands
        waypoints (list): The list to store the waypoints of the path
        i (int): The index to iterate through the waypoints
        x_goal (int): The x coordinate of the goal point
        y_goal (int): The y coordinate of the goal point
        x_start (int): The x coordinate of the start point
        y_start (int): The y coordinate of the start point
        C (int): The clearance value
        rpm1 (int): The rpm of the left wheel
        rpm2 (int): The rpm of the right wheel
        cmd_vel_pub (Publisher): The publisher object to publish to the /cmd_vel topic
        velocity_msg (Twist): The Twist message object to publish the velocity commands
    """

    def __init__(self):
        super().__init__('a_star_controller_node')
        self.subscription = self.create_subscription(Odometry,'/odom',self.odom_callback, 10)
        self.subscription  # prevent unused variable warning

        self.declare_parameter('x_pose',0.0)                                    # declare the parameter x_pose, y_pose, goal_x, goal_y, clearance, rpm1 and rpm2
        self.declare_parameter('y_pose',0.0)
        self.declare_parameter('goal_x',5000.0)
        self.declare_parameter('goal_y',0.0)
        self.declare_parameter('clearance', 50.0)
        self.declare_parameter('rpm1', 20.0)
        self.declare_parameter('rpm2', 30.0)

        self.x_goal = int(self.get_parameter('goal_x').value/10) + 50           # get the goal x and y coordinates and converting to cm
        self.y_goal = int(self.get_parameter('goal_y').value/10) + 100          
        self.x_start = int(self.get_parameter('x_pose').value/10) + 50          # get the start x and y coordinates and converting to cm
        self.y_start = int(self.get_parameter('y_pose').value/10) + 100
        self.C = self.get_parameter('clearance').value/10                       # get the clearance value and converting to cm
        self.rpm1 = self.get_parameter('rpm1').value                            # get the rpm1 and rpm2 values
        self.rpm2 = self.get_parameter('rpm2').value

    
        self.get_logger().info('A start controller node initialised')          # print the message that the node is initialised

        self.a_star_solver()                                                   # call the a_star_solver function

        self.i = 0                                                             # set the index to 0
        self.get_logger().info('Creating Publisher')                           # print the message that the publisher is being created
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)        # create the publisher object to publish to the /cmd_vel topic
        self.velocity_msg = Twist()                                            # create the Twist message object to publish the velocity commands
        

    def odom_callback(self, msg):
        """
        odom_callback function is the callback function for the /odom topic

        Args:
            msg (Odometry): The Odometry message from the /odom topic
            velocity_msg (Twist): The Twist message object to publish the velocity commands
        """
        if (self.i >= len(self.waypoints)):
                self.velocity_msg.linear.x = 0.0                                        # set the linear velocity to 0
                self.velocity_msg.angular.z = 0.0                                       # set the angular velocity to 0
                self.cmd_vel_pub.publish(self.velocity_msg)                             # publish the velocty message
                self.get_logger().info('Goal reached', once=True)                       # print the goal reached message
        else:
            max_rpm = max(self.rpm1, self.rpm2)                                         # get the maximum rpm
            x = msg.pose.pose.orientation.x                                             # get the x,y,z and w coordinates of the robot
            y = msg.pose.pose.orientation.y
            z = msg.pose.pose.orientation.z
            w = msg.pose.pose.orientation.w
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y*y + z*z))                   # convert quaternion to yaw angle
            robot_x = msg.pose.pose.position.x                                           # get the x and y coordinates of the robot
            robot_y = msg.pose.pose.position.y                                     
            wpt_x = self.waypoints[self.i][0]                                           # get the x and y coordinates of the waypoint
            wpt_y = self.waypoints[self.i][1]
            dist = math.sqrt((robot_x-wpt_x)**2 + (robot_y-wpt_y)**2)                   # calculating the distance between the robot and waypoint
            yaw_req = math.atan2(wpt_y-robot_y, wpt_x-robot_x)                          # calculating the required yaw angle
            e = yaw_req - yaw
            """
                max angular velocity of the robot cannot exceed 1.82 rad/s
                max linear velocity of the robot cannot exceed 0.22 m/s
            """
            if dist>0.15:
                self.velocity_msg.angular.z = 0.4*e                              # set the angular velocity
                if self.velocity_msg.angular.z > 1.82:                           # check if the angular velocity is greater than the maximum angular velocity
                    self.velocity_msg.angular.z = 1.82                           # set the angular velocity to the maximum angular velocity   
                elif self.velocity_msg.angular.z < -1.82:                        # check if the angular velocity is less than the minimum angular velocity
                    self.velocity_msg.angular.z = -1.82                          # set the angular velocity to the minimum angular velocity

                self.velocity_msg.linear.x = (2*math.pi*max_rpm/60)*33/1000
                self.cmd_vel_pub.publish(self.velocity_msg)
            else:
                self.get_logger().info('Next Waypoint: %s %s' % (self.waypoints[self.i][0], self.waypoints[self.i][1]))
                self.i += 1


    def a_star_solver(self):
        
        obstacle_set = set()             # set to store the obstacle points
        obstacle_list = []               # list to store the obstacle points in order for videp
        c2c_node_grid = [[float('inf')] * 200 for _ in range(600)]       # create a 2D array for storing cost to come
        tc_node_grid = [[float('inf')] * 200 for _ in range(600)]        # create a 2D array for storing cost to come
        closed_set = set()               # set to store the value of visited and closed points                 
        closed_list = []
        visited={}

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

        valid_goal = False

        while not valid_goal:
            goal = input("Enter the goal coordinates as (x, y) in  mm: ")                         # get the start coordinate and orientaion from the user
            [self.x_goal, self.y_goal] = [int(i) for i in goal.split()]                           # split the input and store the x and y coordinates
            self.x_goal = int(self.x_goal/10) + 50                                                # convert the x coordinate to cm
            self.y_goal = int(self.y_goal/10) + 100                                               # convert the y coordinate to cm
            if (self.x_goal, self.y_goal) in obstacle_set:                                        # check if the goal point is in the obstacle set
                self.get_logger().error("Invalid coordinates, Enter again")                       # print error message
            else:                                        
                valid_goal = True                                                                 # set the flag to true

        self.t = round(((t_max - t_min)*(min_rpm - 75)/(5 - 75)) + t_min, 2)                      # Calculate time step

        def visited_node(node):                                                                   # function to add the visited node to the visited list
            visited.update({node[2]:node[4]})

        def actionnn(node,rpm1,rpm2):
            ul = 2*math.pi*rpm1/60                                                # calculating the left wheel velocity
            ur = 2*math.pi*rpm2/60                                                # calculating the right wheel velocity
            theta_dot = (R/L)*(ul - ur)
            new_heading = (node[5] + (np.rad2deg(theta_dot)*self.t)) % 360        # get the current heading of the robot
            x_vel = (R/2)*(ur+ul)*np.cos(np.deg2rad(new_heading))                 # calculating the x velocity
            y_vel = (R/2)*(ur+ul)*np.sin(np.deg2rad(new_heading))                 # calculating the y velocity
            x = node[4][0] + x_vel*self.t # calculating the new x coordinate
            y = node[4][1] + y_vel*self.t # calculating the new y coordinate
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
            closed_list.append(node[4])        # add the node coordinates to closed set
            closed_set.add(node[4])
            visited_node(node)                 # add the node to the visited list
            index = node[2]                    # store the index of the current node
            parent_index = node[3]             # store the parent index list of current node

            node_dist = math.sqrt((node[4][0]-self.x_goal)**2 + (node[4][1]-self.y_goal)**2)     # calculate the distance between the current node and goal node
            if node_dist < 5:    # if the node is goal position, exit the loop
                self.get_logger().info("Path Successfully Calculated, Moving the robot ...")
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

        path = node[3]
        self.waypoints = []
        for index in path:                                                        # loop to mark the path
            coord=visited[index]                                                  # get the coordinates of the node
            x = (coord[0]-50)/100
            y = (coord[1]-100)/100
            self.waypoints.append((x, y))
        self.waypoints.append(((self.x_goal-50)/100, (self.y_goal-100)/100))        




def main(args=None):
    
    rclpy.init(args=args) # Initialize the ROS client library
    node = AStarControllerNode() # Create an instance of the AStarControllerNode
    try:
        rclpy.spin(node)         # Run the node
    except KeyboardInterrupt:
        node.get_logger().error("KeyboardInterrupt received!") # Print the Keyboard interrupt message
    finally:
        node.destroy_node()      # Destroy the node
        rclpy.shutdown()         # Shutdown the ROS client library


if __name__ == '__main__':
    main()