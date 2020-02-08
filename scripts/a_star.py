#! /usr/bin/env python

import roslib
import rospy
import math 
import random
import sys
import numpy as np

from math import atan2
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler

f = open (sys.argv[1], 'r')
rawFile = f.readlines()
f.close()

current_x = 0.0
current_y = 0.0
current_theta = 0.0
speed = Twist()

class Node():
    
    def __init__(self, parent=None, position=None):
        self.g = 0
        self.h = 0
        self.f = 0 
        
        self.parent = parent
        self.position = position
        
    def __eq__(self, other):
        return self.position == other.position

def readMap():
    global rawFile

    grid = []

    for line in rawFile:
        temp = []
        for item in line:
            if item == '0' or item == '1':
                temp.append(int(item))
        grid.append(temp)
    
    grid = np.array(grid)

    return grid  

def astar (start_loc, end_loc, grid):
    
    open_nodes = []
    close_nodes = []
    
    source_node = Node(None, start_loc)
    goal_node = Node (None, end_loc)
    
    open_nodes.append (source_node)
    
    while len(open_nodes):
        current_node = open_nodes[0]
        curr_ind = 0
        
        for ind, node in enumerate (open_nodes):
            if node.f < current_node.f:
                current_node = node
                curr_ind = ind
                
        open_nodes.pop (curr_ind)
        close_nodes.append(current_node)
        
        curr_pos = current_node.position
        if end_loc == curr_pos:
            print "Reached Goal in A-Star. "
            
            shortest_path = []
            backtrack = current_node
            backtrack = backtrack.parent
            
            while backtrack is not None:
                parent_pos = backtrack.position
                shortest_path.append(parent_pos)
                backtrack = backtrack.parent
                
            return shortest_path
        
        neighbors = []
        for i in range(-1,2,1):
            for j in range(-1,2,1):
                
                neighbor_pos = ( current_node.position[0]+i, current_node.position[1]+j )
                
                # Check if walkable and no obstacle
                # Walkable condition 1
                if grid[ neighbor_pos[0], neighbor_pos[1] ] == 1 or neighbor_pos[0] < 0 or neighbor_pos[1] < 0 or neighbor_pos[0] > grid.shape[0] or neighbor_pos[1] > grid.shape[1]:
                    continue
                
                # Check if neighbour already exists in close_nodes
                flag = 0
                for item in close_nodes:
                    if item.position == neighbor_pos:
                        flag = 1
                        break
                if flag == 1:
                    continue
                
                # Check if consecutive diagonals are continuous 1s
                # Walkable condition 2
                if grid[curr_pos[0]+1, curr_pos[1]]==1 and grid[curr_pos[0],curr_pos[1]+1]==1:
                    continue
                elif grid[curr_pos[0]-1, curr_pos[1]]==1 and grid[curr_pos[0],curr_pos[1]+1]==1:
                    continue
                elif grid[curr_pos[0]-1, curr_pos[1]]==1 and grid[curr_pos[0],curr_pos[1]-1]==1:
                    continue
                elif grid[curr_pos[0]+1, curr_pos[1]]==1 and grid[curr_pos[0],curr_pos[1]-1]==1:
                    continue
                # Else add neighbor to list of walkable neighbors
                else:
                    neighbor_node = Node (current_node, neighbor_pos)
                    
                    if i == -1 or i == 1:
                        if j == -1 or j == 1:
                            neighbor_node.g = 1.4
                    else:
                        neighbor_node.g = 1.0
                        
                    neighbors.append(neighbor_node)

        
        # Calculate the shortest path
        for child in neighbors:

            child.g = current_node.g + child.g
            child.h = ( ( child.position[0] - end_loc[0] )**2  + ( child.position[1] - end_loc[1] )**2 )
            
            epsilon = 1
            child.f = child.g + epsilon*child.h
            
            for item in open_nodes:
                if item == child:
                    if item.g < child.g:
                        continue
            
            open_nodes.append(child)
        

def planning():
    
    grid = readMap()

    grid_list = grid
    grid = np.array (grid)
    print (grid)

    start_xy = (-8, -2)
    g_x = rospy.get_param('goalx')
    g_y = rospy.get_param('goaly')
    
    print "\nStart in world map: "
    print (start_xy)
    
    print "\nEnd in world map: "
    print "(",g_x,", ", g_y,")"

    origin = ( int(grid.shape[0]/2), int(grid.shape[1]/2) )
    print "origin of grid= ", origin

    start = ( int(origin[0] - start_xy[1]) , int (origin[1] + start_xy[0]) )

    end = ( int(origin[0] - g_y) , int (origin[1] + g_x) )

    print "\nStart location in grid: "
    print (start)
    print "\nEnd location in grid: "
    print (end)
    
    path = astar(start, end, grid)
    path.reverse()
    path.append(end)
    print "\nComputed A-Star path in grid: " 
    print (path)
    
    new_maze = np.ones(grid.shape)

    world_path = []
    route = []
    for item in path:    
        w_Y = origin[0] - item[0]
        w_X = item[1] - origin[1] # - item[1]
        world_path.append((w_X, w_Y))

        new_maze[item[0], item[1]] = 0
        
        x = int (item[0])
        y = int (item[1])
        route.append([x, y])

    print "\nA-star path XY coordinates:\n", world_path
    print "\n"

    print  "\nA-Star Movement pattern represented by 0s. \n\n", new_maze
    return world_path

def find_Slope (x1, x2, y1, y2):
    
    m = int ( (y2 - y1)/ (x2 -x1) )
    c = y2 - m*x2

    return m, c 

def sensor_Read(Laser):     #callback function definition by subscribing from /base_scan topic
    global min_range
    min_range = min (Laser.ranges[100:241] )

def newposition(msg):
    global current_x
    global current_y
    global current_theta
    
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    
    quaternion = msg.pose.pose.orientation
    (roll, pitch, current_theta) = euler_from_quaternion ([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

def movement (route):
    
    path = route
    #Subscribing to robot's current position
    rospy.Subscriber('base_pose_ground_truth', Odometry, newposition)
    
    #Subscribing to base_scan to read the laser values
    rospy.Subscriber('base_scan', LaserScan, sensor_Read)

    #Defining the publisher to send the linear and angular velocity values to the robot over cmd_vel topic
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    r = rospy.Rate(1)

    # remove the starting coordinate from the a-star path
    path_len = len (route) - 1 
    goal = route[ path_len ]
    path.pop(0)

    print "\n\nPlease Wait.............................. Robot will take time to move. "
    print "\n\n...................................Robot has started moving................................"
    # start the movement on the path
    while not rospy.is_shutdown():
    
    # Step1: First orient the robot in the direction of tte goal
        inc_x = goal[0] - current_x
        inc_y = goal[1] - current_y

        angle_to_goal = atan2 (inc_y, inc_x)
        
        # rotate the robot in the direction of the goal
        if abs(angle_to_goal - current_theta) > 0.2:
            speed.angular.z = 1.5
            speed.linear.x = 0
            pub.publish(speed)
        else:
            speed.angular.z = 0
            pub.publish (speed)
        
        # print "\n\n...................................Robot has started moving................................"
        # read the a-star path
        while len(path)>0:
            
            ######### check for every location in the path ########
            # seek the first point from the a-star path
            item = path[0]

            #calculate the difference in x and y coordinates of current and next point
            dir_x = item[0] - current_x + 0.5       # added 0.5 to x-coordinate to incorporate buffer in a-star grid and current coord
            dir_y = item[1] - current_y - 0.5       # subtracted 0.5 from y-coordinate to incorporate buffer in a-star grid and current coord

            # calculate angle of orientation between current and next
            rot_angle = atan2 (dir_y, dir_x)

            # check if orientation of rotation is clockwise
            if rot_angle - current_theta > 0.5:
                speed.angular.z = 0.5
                speed.linear.x = 0
                pub.publish (speed)
            # check if orientation of rotation is anti-clockwise
            elif current_theta - rot_angle > 0.2:
                speed.angular.z = -0.5
                speed.linear.x = 0
                pub.publish (speed)
            
            # if no rotation is required, move linealry / straight, avoiding the obstacles
            else:
                # check if there's an obstacle infront, so rotate and avoid obstacle
                if min_range < 0.5:
                    speed.angular.z = -2.0
                    speed.linear.x = 0.5
                    pub.publish(speed)
                # check if no obstacle, keep moving straight
                else:
                    speed.linear.x = 1.0
                    pub.publish (speed)
            # check if robot reached the current point from the path. Stop the movement
            # Remove the current point from the list of points in the path
            if (dir_x > -0.1 and dir_x < 0.1) and (dir_y > -0.1 and dir_y < 0.1):
                speed.linear.x = 0
                speed.angular.z = 0
                pub.publish (speed)
                path.pop(0)
        # print "\n\nReached goal."
            
        r.sleep()
        if len (path) == 0:
            print "\n\nRobot has reached goal."
            break
    # print "\n\nRobot has reached goal."        

if __name__ == '__main__':
    try:
        rospy.init_node('aStar', anonymous = True)

        route = planning()

        movement(route)

        print "\n\n"
        print "All execution successfully completed............................................ E X I T I N G !!!"
        exit()
    except rospy.ROSInterruptException:
        pass
