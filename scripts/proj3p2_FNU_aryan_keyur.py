# ENPM 661: Planning for Robotics
# Project 3 - Phase 2
# authors: Keyur Borad, Aryan Mishra, FNU Koustubh

# import libraries

import cv2
import numpy as np
import heapq as hq
import math
import time

canvas = np.ones((200,600,3))   # creating a frame for video generation
obstacle_set = set()             # set to store the obstacle points
obstacle_list = []               # list to store the obstacle points in order for videp

c2c_node_grid = [[float('inf')] * 200 for _ in range(600)]       # create a 2D array for storing cost to come
tc_node_grid = [[float('inf')] * 200 for _ in range(600)]        # create a 2D array for storing cost to come
closed_set = set()               # set to store the value of visited and closed points                 
closed_list = []
visited={}

C = int(input("Enter the clearance from the obstacle in mm: "))     # Get clearance from the user
C = C/10

# All units are in cm
R = 66/20                                                  # Robot wheel radius
r = 22                                                   # Robot radius
L = 28.7                                                   # Robot wheel track
T = C + r                                                 # Total clearance

t_max = 3.5
t_min = 0.25

x_goal = 0  # Initialize the goal x coordinate
y_goal = 0  # Initialize the goal y coordinate
x_start = 0 # Initialize the start x coordinate
y_start = 0 # Initialize the start y coordinate

# Funtion to update the visted nodes
def visited_node(node):
    visited.update({node[2]:node[4]})


'''
Loop to define the obstacle points in the map
'''
for y in range(200):                                       # loop to define the obstacle points : x
    for x in range(600):                                  # loop to define the obstacle points : y
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

valid_rpm = False

while not valid_rpm:
    rpm = input("Enter the RPM1 and RPM2: ")
    [rpm1, rpm2] = [int(i) for i in rpm.split()]
    if (5<=rpm1<=75 and 5<=rpm2<=75):
        valid_rpm = True
    else:
        print("Invalid rpm, Enter again in the range of 5 and 75")

min_rpm = min(rpm1,rpm2)

t = round(((t_max - t_min)*(min_rpm - 75)/(5 - 75)) + t_min, 2)                     # Calculate time step
print("Calculated time step: ", t)

def action_1(node):
    ul = 0
    ur = 2*math.pi*rpm1/60
    new_heading = (node[5] + np.rad2deg(((R/L)*(ul - ur)*t))) % 360        # get the current heading of the robot
    x_vel = (R/2)*(ur+ul)*np.cos(np.deg2rad(new_heading))
    y_vel = (R/2)*(ur+ul)*np.sin(np.deg2rad(new_heading))
    x = node[4][0] + x_vel*t # calculate the new x coordinate
    y = node[4][1] + y_vel*t # calculate the new y coordinate
    x = round(x) 
    y = round(y)
    c2c = node[1] + math.sqrt((x_vel*t)**2 + (y_vel*t)**2)                                     # calculate the cost to come
    c2g = math.sqrt((y_goal-y)**2 + (x_goal-x)**2)   # calculate the cost to goal
    tc = c2c + c2g                                   # calculate the total cost
    return (x,y),new_heading,tc,c2c                  # return the new node's coordinates, heading, total cost and cost to come


def action_2(node):
    ul = 2*math.pi*rpm1/60
    ur = 0
    new_heading = (node[5] + np.rad2deg(((R/L)*(ul - ur)*t))) % 360        # get the current heading of the robot
    x_vel = (R/2)*(ur+ul)*np.cos(np.deg2rad(new_heading))
    y_vel = (R/2)*(ur+ul)*np.sin(np.deg2rad(new_heading))
    x = node[4][0] + x_vel*t # calculate the new x coordinate
    y = node[4][1] + y_vel*t # calculate the new y coordinate
    x = round(x) 
    y = round(y)
    c2c = node[1] + math.sqrt((x_vel*t)**2 + (y_vel*t)**2)                                     # calculate the cost to come
    c2g = math.sqrt((y_goal-y)**2 + (x_goal-x)**2)   # calculate the cost to goal
    tc = c2c + c2g                                   # calculate the total cost
    return (x,y),new_heading,tc,c2c                  # return the new node's coordinates, heading, total cost and cost to come


def action_3(node):
    ul = 2*math.pi*rpm1/60
    ur = 2*math.pi*rpm1/60
    new_heading = (node[5] + np.rad2deg(((R/L)*(ul - ur)*t))) % 360        # get the current heading of the robot
    x_vel = (R/2)*(ur+ul)*np.cos(np.deg2rad(new_heading))
    y_vel = (R/2)*(ur+ul)*np.sin(np.deg2rad(new_heading))
    x = node[4][0] + x_vel*t # calculate the new x coordinate
    y = node[4][1] + y_vel*t # calculate the new y coordinate
    x = round(x) 
    y = round(y)
    c2c = node[1] + math.sqrt((x_vel*t)**2 + (y_vel*t)**2)                                  # calculate the cost to come
    c2g = math.sqrt((y_goal-y)**2 + (x_goal-x)**2)   # calculate the cost to goal
    tc = c2c + c2g                                   # calculate the total cost
    return (x,y),new_heading,tc,c2c                  # return the new node's coordinates, heading, total cost and cost to come


def action_4(node):
    ul = 0
    ur = 2*math.pi*rpm2/60
    new_heading = (node[5] + np.rad2deg(((R/L)*(ul - ur)*t))) % 360        # get the current heading of the robot
    x_vel = (R/2)*(ur+ul)*np.cos(np.deg2rad(new_heading))
    y_vel = (R/2)*(ur+ul)*np.sin(np.deg2rad(new_heading))
    x = node[4][0] + x_vel*t # calculate the new x coordinate
    y = node[4][1] + y_vel*t # calculate the new y coordinate
    x = round(x) 
    y = round(y)
    c2c = node[1] + math.sqrt((x_vel*t)**2 + (y_vel*t)**2)                                     # calculate the cost to come
    c2g = math.sqrt((y_goal-y)**2 + (x_goal-x)**2)   # calculate the cost to goal
    tc = c2c + c2g                                   # calculate the total cost
    return (x,y),new_heading,tc,c2c                  # return the new node's coordinates, heading, total cost and cost to come


def action_5(node):
    ul = 2*math.pi*rpm2/60
    ur = 0
    new_heading = (node[5] + np.rad2deg(((R/L)*(ul - ur)*t))) % 360        # get the current heading of the robot
    x_vel = (R/2)*(ur+ul)*np.cos(np.deg2rad(new_heading))
    y_vel = (R/2)*(ur+ul)*np.sin(np.deg2rad(new_heading))
    x = node[4][0] + x_vel*t # calculate the new x coordinate
    y = node[4][1] + y_vel*t # calculate the new y coordinate
    x = round(x) 
    y = round(y)
    c2c = node[1] + math.sqrt((x_vel*t)**2 + (y_vel*t)**2)                                     # calculate the cost to come
    c2g = math.sqrt((y_goal-y)**2 + (x_goal-x)**2)   # calculate the cost to goal
    tc = c2c + c2g                                   # calculate the total cost
    return (x,y),new_heading,tc,c2c                  # return the new node's coordinates, heading, total cost and cost to come


def action_6(node):
    ul = 2*math.pi*rpm2/60
    ur = 2*math.pi*rpm2/60
    new_heading = (node[5] + np.rad2deg(((R/L)*(ul - ur)*t))) % 360        # get the current heading of the robot
    x_vel = (R/2)*(ur+ul)*np.cos(np.deg2rad(new_heading))
    y_vel = (R/2)*(ur+ul)*np.sin(np.deg2rad(new_heading))
    x = node[4][0] + x_vel*t # calculate the new x coordinate
    y = node[4][1] + y_vel*t # calculate the new y coordinate
    x = round(x) 
    y = round(y)
    c2c = node[1] + math.sqrt((x_vel*t)**2 + (y_vel*t)**2)                                   # calculate the cost to come
    c2g = math.sqrt((y_goal-y)**2 + (x_goal-x)**2)   # calculate the cost to goal
    tc = c2c + c2g                                   # calculate the total cost
    return (x,y),new_heading,tc,c2c                  # return the new node's coordinates, heading, total cost and cost to come


def action_7(node):
    ul = 2*math.pi*rpm1/60
    ur = 2*math.pi*rpm2/60
    new_heading = (node[5] + np.rad2deg(((R/L)*(ul - ur)*t))) % 360        # get the current heading of the robot
    x_vel = (R/2)*(ur+ul)*np.cos(np.deg2rad(new_heading))
    y_vel = (R/2)*(ur+ul)*np.sin(np.deg2rad(new_heading))
    x = node[4][0] + x_vel*t # calculate the new x coordinate
    y = node[4][1] + y_vel*t # calculate the new y coordinate
    x = round(x) 
    y = round(y)
    c2c = node[1] + math.sqrt((x_vel*t)**2 + (y_vel*t)**2)                                    # calculate the cost to come
    c2g = math.sqrt((y_goal-y)**2 + (x_goal-x)**2)   # calculate the cost to goal
    tc = c2c + c2g                                   # calculate the total cost
    return (x,y),new_heading,tc,c2c                  # return the new node's coordinates, heading, total cost and cost to come


def action_8(node):
    ul = 2*math.pi*rpm2/60
    ur = 2*math.pi*rpm1/60
    new_heading = (node[5] + np.rad2deg(((R/L)*(ul - ur)*t))) % 360        # get the current heading of the robot
    x_vel = (R/2)*(ur+ul)*np.cos(np.deg2rad(new_heading))
    y_vel = (R/2)*(ur+ul)*np.sin(np.deg2rad(new_heading))
    x = node[4][0] + x_vel*t # calculate the new x coordinate
    y = node[4][1] + y_vel*t # calculate the new y coordinate
    x = round(x) 
    y = round(y)
    c2c = node[1] + math.sqrt((x_vel*t)**2 + (y_vel*t)**2)                                   # calculate the cost to come
    c2g = math.sqrt((y_goal-y)**2 + (x_goal-x)**2)   # calculate the cost to goal
    tc = c2c + c2g                                   # calculate the total cost
    return (x,y),new_heading,tc,c2c                  # return the new node's coordinates, heading, total cost and cost to come



start_time = time.time()  
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

    node_dist = math.sqrt((node[4][0]-x_goal)**2 + (node[4][1]-y_goal)**2)     # calculate the distance between the current node and goal node
    if node_dist < 5:    # if the node is goal position, exit the loop
        print("Goal reached")
        break

    point, new_heading, tc, c2c = action_1(node)
    # print("Action 1", point, new_heading, tc, c2c)
    if point not in obstacle_set and point not in closed_set and 0<=x<=600 and 0<=y<=200:           # check if the new node is in the obstacle set or visited list
        x = point[0]                                                    # get the x coordinate of the new node
        y = point[1]                                                    # get the y coordinate of the new node
        if tc<tc_node_grid[x][y]:                                       # check if the new cost to come is less than original cost to come
            new_parent_index = parent_index.copy()                      # copy the parent index list of the current node
            new_parent_index.append(index)                              # Append the current node's index to the new node's parent index list
            new_index+=1                                                # increment the index
            tc_node_grid[x][y] = tc                                     # Update the new total cost
            c2c_node_grid[x][y] = c2c                                   # Update the new cost to come
            new_node = (tc, c2c, new_index, new_parent_index, point, new_heading) # create the new node
            hq.heappush(open_list, new_node)                            # push the new node to the open list

    point, new_heading, tc, c2c = action_2(node)
    # print("Action 2", point, new_heading, tc, c2c)
    if point not in obstacle_set and point not in closed_set and 0<=x<=600 and 0<=y<=200:           # check if the new node is in the obstacle set or visited list
        x = point[0]                                                    # get the x coordinate of the new node
        y = point[1]                                                    # get the y coordinate of the new node
        if tc<tc_node_grid[x][y]:                                       # check if the new cost to come is less than original cost to come
            new_parent_index = parent_index.copy()                      # copy the parent index list of the current node
            new_parent_index.append(index)                              # Append the current node's index to the new node's parent index list
            new_index+=1                                                # increment the index
            tc_node_grid[x][y] = tc                                     # Update the new total cost
            c2c_node_grid[x][y] = c2c                                   # Update the new cost to come
            new_node = (tc, c2c, new_index, new_parent_index, point, new_heading) # create the new node
            hq.heappush(open_list, new_node)                            # push the new node to the open list

    # print("Action 3", point, new_heading, tc, c2c)
    point, new_heading, tc, c2c = action_3(node)
    if point not in obstacle_set and point not in closed_set and 0<=x<=600 and 0<=y<=200:           # check if the new node is in the obstacle set or visited list
        x = point[0]                                                    # get the x coordinate of the new node
        y = point[1]                                                    # get the y coordinate of the new node
        if tc<tc_node_grid[x][y]:                                       # check if the new cost to come is less than original cost to come
            new_parent_index = parent_index.copy()                      # copy the parent index list of the current node
            new_parent_index.append(index)                              # Append the current node's index to the new node's parent index list
            new_index+=1                                                # increment the index
            tc_node_grid[x][y] = tc                                     # Update the new total cost
            c2c_node_grid[x][y] = c2c                                   # Update the new cost to come
            new_node = (tc, c2c, new_index, new_parent_index, point, new_heading) # create the new node
            hq.heappush(open_list, new_node)                            # push the new node to the open list

    # print("Action 4", point, new_heading, tc, c2c)
    point, new_heading, tc, c2c = action_4(node)
    if point not in obstacle_set and point not in closed_set and 0<=x<=600 and 0<=y<=200:           # check if the new node is in the obstacle set or visited list
        x = point[0]                                                    # get the x coordinate of the new node
        y = point[1]                                                    # get the y coordinate of the new node
        if tc<tc_node_grid[x][y]:                                       # check if the new cost to come is less than original cost to come
            new_parent_index = parent_index.copy()                      # copy the parent index list of the current node
            new_parent_index.append(index)                              # Append the current node's index to the new node's parent index list
            new_index+=1                                                # increment the index
            tc_node_grid[x][y] = tc                                     # Update the new total cost
            c2c_node_grid[x][y] = c2c                                   # Update the new cost to come
            new_node = (tc, c2c, new_index, new_parent_index, point, new_heading) # create the new node
            hq.heappush(open_list, new_node)                            # push the new node to the open list

    # print("Action 5", point, new_heading, tc, c2c)
    point, new_heading, tc, c2c = action_5(node)
    if point not in obstacle_set and point not in closed_set and 0<=x<=600 and 0<=y<=200:           # check if the new node is in the obstacle set or visited list
        x = point[0]                                                    # get the x coordinate of the new node
        y = point[1]                                                    # get the y coordinate of the new node
        if tc<tc_node_grid[x][y]:                                       # check if the new cost to come is less than original cost to come
            new_parent_index = parent_index.copy()                      # copy the parent index list of the current node
            new_parent_index.append(index)                              # Append the current node's index to the new node's parent index list
            new_index+=1                                                # increment the index
            tc_node_grid[x][y] = tc                                     # Update the new total cost
            c2c_node_grid[x][y] = c2c                                   # Update the new cost to come
            new_node = (tc, c2c, new_index, new_parent_index, point, new_heading) # create the new node
            hq.heappush(open_list, new_node)                            # push the new node to the open list

    # print("Action 6", point, new_heading, tc, c2c)
    point, new_heading, tc, c2c = action_6(node)
    if point not in obstacle_set and point not in closed_set and 0<=x<=600 and 0<=y<=200:           # check if the new node is in the obstacle set or visited list
        x = point[0]                                                    # get the x coordinate of the new node
        y = point[1]                                                    # get the y coordinate of the new node
        if tc<tc_node_grid[x][y]:                                       # check if the new cost to come is less than original cost to come
            new_parent_index = parent_index.copy()                      # copy the parent index list of the current node
            new_parent_index.append(index)                              # Append the current node's index to the new node's parent index list
            new_index+=1                                                # increment the index
            tc_node_grid[x][y] = tc                                     # Update the new total cost
            c2c_node_grid[x][y] = c2c                                   # Update the new cost to come
            new_node = (tc, c2c, new_index, new_parent_index, point, new_heading) # create the new node
            hq.heappush(open_list, new_node)                            # push the new node to the open list

    # print("Action 7", point, new_heading, tc, c2c)
    point, new_heading, tc, c2c = action_7(node)
    if point not in obstacle_set and point not in closed_set and 0<=x<=600 and 0<=y<=200:           # check if the new node is in the obstacle set or visited list
        x = point[0]                                                    # get the x coordinate of the new node
        y = point[1]                                                    # get the y coordinate of the new node
        if tc<tc_node_grid[x][y]:                                       # check if the new cost to come is less than original cost to come
            new_parent_index = parent_index.copy()                      # copy the parent index list of the current node
            new_parent_index.append(index)                              # Append the current node's index to the new node's parent index list
            new_index+=1                                                # increment the index
            tc_node_grid[x][y] = tc                                     # Update the new total cost
            c2c_node_grid[x][y] = c2c                                   # Update the new cost to come
            new_node = (tc, c2c, new_index, new_parent_index, point, new_heading) # create the new node
            hq.heappush(open_list, new_node)                            # push the new node to the open list

    # print("Action 8", point, new_heading, tc, c2c)   
    point, new_heading, tc, c2c = action_8(node)
    if point not in obstacle_set and point not in closed_set and 0<=x<=600 and 0<=y<=200:           # check if the new node is in the obstacle set or visited list
        x = point[0]                                                    # get the x coordinate of the new node
        y = point[1]                                                    # get the y coordinate of the new node
        if tc<tc_node_grid[x][y]:                                       # check if the new cost to come is less than original cost to come
            new_parent_index = parent_index.copy()                      # copy the parent index list of the current node
            new_parent_index.append(index)                              # Append the current node's index to the new node's parent index list
            new_index+=1                                                # increment the index
            tc_node_grid[x][y] = tc                                     # Update the new total cost
            c2c_node_grid[x][y] = c2c                                   # Update the new cost to come
            new_node = (tc, c2c, new_index, new_parent_index, point, new_heading) # create the new node
            hq.heappush(open_list, new_node)                            # push the new node to the open list
    # break
    
print((node[4][0]-50)*10, (node[4][1]-100)*10)

# # Mark the obstacle points in the frame, including points after bloating
for point in obstacle_list:                            # loop to mark the obstacle points
    canvas[point[1],point[0]] = [255, 0, 0]            # mark the obstacle points with blue color

# Draw the obstacles in the frame, excluding the points after bloating
cv2.rectangle(canvas, (150, 200), (175, 100), (0 , 0, 255), -1)   # draw the first rectangle
cv2.rectangle(canvas, (250, 100), (275, 0), (0 , 0, 255), -1)     # draw the second rectangle
cv2.circle(canvas,(420, 120), 60, (0,0,255),-1)            # draw the circle shaped obstacle
cv2.circle(canvas,(x_start, y_start), 5, (0,0,255), -1)             # mark the goal point with red color
cv2.circle(canvas,(x_goal, y_goal), 5, (255,0,255), -1)             # mark the goal point with red color
cv2.circle(canvas, node[4] , 5, (0,255,255), -1)             # mark the goal point with red color


path = node[3]            # Get the parent node list 
counter = 0               # counter to count the frames to write on video

fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for MP4 format
video_writer = cv2.VideoWriter('output/output.mp4', fourcc, 60, (600, 200)) # Video writer object

'''
Loop to mark the explored nodes in order on the frame
'''
print("Exploring map")


# print(closed_set)
for node in closed_list:                                                  # loop to mark the explored nodes

    # canvas[node[1], node[0]] = [0, 255, 0]                               # mark the explored nodes with green color
    cv2.circle(canvas, node, 1, (0,255,0), -1)             # mark the goal point with red color
    counter +=1                                                          # increment the counter
    if counter%25 == 0 or counter == 0:                                 # check if the counter is divisible by 500
        canvas_resized = cv2.resize(canvas, (1500, 500))    
        canvas_flipped = cv2.flip(canvas_resized, 0)
        canvas_flipped = cv2.flip(canvas,0)                              # flip the frame
        canvas_flipped_uint8 = cv2.convertScaleAbs(canvas_flipped)       # convert the frame to uint8
        # cv2.imshow('window',canvas_flipped_uint8)
        # cv2.waitKey(1)
        video_writer.write(canvas_flipped_uint8)                         # write the frame to video

'''
Loop to mark the path created
'''
print("Backtracking")

for index in path:                                                        # loop to mark the path
    coord=visited[index]                                                  # get the coordinates of the node
    
    cv2.circle(canvas, (coord[0],coord[1]), 1, [0,0,0], -1)               # mark the path with black color 
    canvas_flipped = cv2.flip(canvas, 0)
    canvas_flipped_uint8 = cv2.convertScaleAbs(canvas_flipped)            # convert the frame to uint8
    # cv2.imshow('window',canvas_flipped_uint8)
    # cv2.waitKey(1)

    video_writer.write(canvas_flipped_uint8)                              # write the frame to video



'''
Loop to add some additional frames at the end of the video
'''
for i in range(150):
    video_writer.write(canvas_flipped_uint8)                               # write the frame to video
    
print("Video Processed")                                                   # print message

video_writer.release()                                                     # release the video writer

end_time = time.time()                                                     # get the end time of the program
print(f"The runtime of my program is {end_time - start_time} seconds.")    # print the runtime of the program

cv2.imshow("canvas", canvas_flipped_uint8)
cv2.waitKey(0)
cv2.destroyAllWindows()
