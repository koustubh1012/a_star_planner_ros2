#ENPM 661: Planning for Robotics
#Project 3 - Phase 2
#authors: Keyur Borad, Aryan Mishra, FNU Koustubh

# import libraries

import cv2
import numpy as np
import heapq as hq
import math
import time

canvas = np.ones((2001,6001,3))   # creating a frame for video generation
obstacle_set = set()             # set to store the obstacle points
obstacle_list = []               # list to store the obstacle points in order for videp

c2c_node_grid = [[float('inf')] * 2000 for _ in range(6000)]       # create a 2D array for storing cost to come
tc_node_grid = [[float('inf')] * 2000 for _ in range(6000)]        # create a 2D array for storing cost to come
closed_set = []               # set to store the value of visited and closed points                 
closed_list = np.zeros((6000, 2000, 12))
visited={}

'''
Loop to define the obstacle points in the map
'''
C = int(input("Enter the clearance from the obstacle in pixel: "))     # Get clearance from the user
R = int(input("Enter the radius of the robot in pixel: "))             # Get the robot radius from user


x_goal = 0  # Initialize the goal x coordinate
y_goal = 0  # Initialize the goal y coordinate
x_start = 0 # Initialize the start x coordinate
y_start = 0 # Initialize the start y coordinate

# Funtion to update the visted nodes
def visited_node(node):
    visited.update({node[2]:node[4]})

T = C + R

for y in range(2000):                                       # loop to define the obstacle points : x
    for x in range(6000):                                  # loop to define the obstacle points : y
        canvas[y,x] = [255,255,255]                        # mark the points in the frame with white color
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
        elif (2000-T<=y<2000):                               # points in the top boundary
            obstacle_set.add((x,y))                        # add the points to the obstacle set
            obstacle_list.append((x,y))                    # add the points to the obstacle list
            c2c_node_grid[x][y] = -1                       # mark the points in the cost to come grid with -1
            tc_node_grid[x][y] = -1                        # mark the points in the total cost grid with -1
        elif (6000-T<=x<6000):                             # points in the right boundary
            obstacle_set.add((x,y))                        # add the points to the obstacle set
            obstacle_list.append((x,y))                    # add the points to the obstacle list
            c2c_node_grid[x][y] = -1                       # mark the points in the cost to come grid with -1
            tc_node_grid[x][y] = -1                        # mark the points in the total cost grid with -1
        
        elif (1500-T<=x<=1750+T) and (1000-T<=y<=2000):    # points in first rectangle
            obstacle_set.add((x,y))                        # add the points to the obstacle set
            obstacle_list.append((x,y))                    # add the points to the obstacle list
            c2c_node_grid[x][y] = -1                       # mark the points in the cost to come grid with -1
            tc_node_grid[x][y] = -1                        # mark the points in the total cost grid with -1

        elif (2500-T<=x<=2750+T) and (0<=y<=1000+T):       # points in second rectangle
            obstacle_set.add((x,y))                        # add the points to the obstacle set
            obstacle_list.append((x,y))                    # add the points to the obstacle list
            c2c_node_grid[x][y] = -1                       # mark the points in the cost to come grid with -1
            tc_node_grid[x][y] = -1                        # mark the points in the total cost grid with -1
         
        # Points int the Circle shaped obstacle
        elif ((x-4200)**2 + (y-1200)**2 <= (600+T)**2):      # points in the first rectangle of Concave shaped obstacle
            obstacle_set.add((x,y))                        # add the points to the obstacle set
            obstacle_list.append((x,y))                    # add the points to the obstacle list
            c2c_node_grid[x][y] = -1                       # mark the points in the cost to come grid with -1
            tc_node_grid[x][y] = -1                        # mark the points in the total cost grid with -1
      
valid_start = False                                                             # flag to check if the start point is valid

while not valid_start:                                                          # loop to check if the start point is valid
        x_start = int(input("Enter the start x coordinate: "))                  # get the start x coordinate from the user
        y_start = int(input("Enter the start y coordinate: "))                  # get the start y coordinate from the user
        theta_start = int(input("Enter the initial heading: "))                 # get the start heading from the user
        x_start += 500
        y_start += 1000
        if (x_start, y_start) in obstacle_set:                                  # check if the start point is in the obstacle set
            print("Invalid coordinates, Enter again: ")                         # print error message
        else:
            initial_node = (0, 0, 1, [], (x_start, y_start), theta_start)       # create the initial node
            valid_start = True                                                  # set the flag to true

# valid_goal = False                                                              # flag to check if the goal point is valid

# while not valid_goal:
#         x_goal = int(input("Enter the goal x coordinate: "))                   # get the goal x coordinate from the user
#         y_goal = int(input("Enter the goal y coordinate: "))                   # get the goal y coordinate from the user
#         theta_goal = int(input("Enter the goal heading: "))                    # get the goal heading from the user
#         if (x_goal, y_goal) in obstacle_set:                                   # check if the goal point is in the obstacle set
#             print("Invalid coordinates, Enter again: ")                        # print error message
#         else:
#             goal = (x_goal, y_goal)                                            # create the goal node
#             valid_goal = True                                                  # set the flag to true

# valid_step = False                                                             # flag to check if the step size is valid

# while not valid_step:                                                          # loop to check if the step size is valid
#         L = int(input("Enter step size: "))                                    # get the step size from the user
#         if not (1 <= L <= 10):                                                 # check if the step size is between 1 and 10
#             print("Invalid Step Size, Enter Again: ")                          # print error message
#         else:
#             valid_step = True                                                  # set the flag to true
start_time = time.time()  
new_index = 1         
open_list = []
# hq.heappush(open_list,initial_node)        # Push initial node to the list
# hq.heapify(open_list)                      # covers list to heapq data type

# Mark the obstacle points in the frame, including points after bloating
for point in obstacle_list:                            # loop to mark the obstacle points
    canvas[point[1],point[0]] = [255, 0, 0]            # mark the obstacle points with blue color

# Draw the obstacles in the frame, excluding the points after bloating
cv2.rectangle(canvas, (1500, 2000), (1750, 1000), (0 , 0, 255), -1)   # draw the first rectangle
cv2.rectangle(canvas, (2500, 1000), (2750, 0), (0 , 0, 255), -1)     # draw the second rectangle
cv2.circle(canvas,(4200, 1200), 600, (0,0,255),-1)            # draw the circle shaped obstacle
cv2.circle(canvas,(x_start, y_start), 30, (0,0,255), -1)             # mark the goal point with red color

canvas_resized = cv2.resize(canvas, (1500, 500))    
canvas_resized = cv2.flip(canvas_resized, 0)
cv2.imshow("canvas", canvas_resized)
cv2.waitKey(0)
cv2.destroyAllWindows()
