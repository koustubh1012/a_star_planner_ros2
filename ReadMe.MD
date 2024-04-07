# ENPM 661: Planning for Robotics
## PROJECT 3 - PHASE 2

### Authors:
- Keyur Borad (kborad@umd.edu)- 120426049
- Aryan Mishra (amishr17@umd.edu)- 120106521
- FNU Koustubh (koustubh@umd.edu)- 120273755

### Library used:
- cv2
- Numpy
- heapq
- math
- time

### Instructions to Run the Code:
1. Download the package and navigate to your directory.
    ```
    cd YOUR_DIRECTORY
    ```
2. The package has two folders Part01 and Part02

   To run Part01:

    a) Run the following command:
    ```
    python proj3p2_FNU_aryan_keyur.py
    ```
    b) The following attributes are required to be entered via CLI:
    ```
    Enter the clearance from the obstacle in mm:
    Enter the start coordinates and orientation as (x, y, theta):
    Enter the goal coordinates as (x,y) :

    ```
    c) After completion, open your folder. There would be a simulation video titled `new_output.mp4`.

    To run Part02:
    
    a) Open your ROS Workspace and place the Part02 folder in your src directory.
    ```
    cd YOUR_ROS_WORKSPACE
    cd src
    Place your ROS2 pkg Part02 here.

    ```
    b) Inside the Part02 folder , copy the turtlebot3_project3 folder outside inside your src directory.
    ```
    Place the turtlebot3_project3 folder inside src folder such that the following structure would be visible:
    YOUR_ROS_WORKSPACE/src/turtlebot3_project3 
    ```
    c) Revert back out of src and run the command:
    ```
    colcon build --packages-select turtlebot3_project3
    ```
    d) Source your bash script
    ```
    source install/setup.bash
    ```
    e) Open another terminal and source your script
    f) In your first terminal, to start the competition_world, run the following command:
    ```
    ros2 launch turtlebot3_project3 competition_world.launch.py
    ```
    g) In your second terminal, run the following node:
    ```
    ros2 run turtlebot3_project3 a_star_closed_loop.py
    ```

### Github Repository
[https://github.com/koustubh1012/a_star_planner_ros2](https://github.com/koustubh1012/a_star_planner_ros2)
