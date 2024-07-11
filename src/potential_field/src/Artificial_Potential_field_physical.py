#!/usr/bin/env python

import rospy
import sys
import geometry_msgs.msg as gmsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import math
import numpy as np


class VelocityController():
    def __init__(self, x_goal, y_goal):
        # User defined parameters

        self.Vmax = 0.15 # Max velocity Limit for the robot
        self.kp_l1 = 0.01  # Linear Controller proportional gain1
        self.kp_l2 = 0.05  # Linear Controller proportional gain2
        self.kp_l3 = 0.0015  # Linear Controller proportional gain1
        self.kp_w_1 = 0.4 * 3/4  # Angular Controller proportional gain1
        self.kp_w_2 = 1.25 * 3/4 # Angular Controler proportional gain2
        # Position
        self.pos = gmsg.Point()
        #Set the initial positional values of the bot to zero.
        self.pos.x = 0.0
        self.pos.y = 0.0
        self.theta = 0.0

        # Assign the goal coordinates
        self.x_goal = x_goal
        self.y_goal = y_goal

        # Attractive and Repulsive force params
        self.K_a = 50      # Attractive force proportional constant
        self.K_r = 0.625       # Repulsive force proportional constant
        self.rr = 2        # Region of influence of the Repulsive force
        self.x_r = 0.0     #Resultant repulsive force vector in X direction
        self.y_r = 0.0     #Resultant repulsive force vector in Y direction
        self.D_obs = 0.3   #safe region distance any obstacle must maintain from the robot in its FOV.
        self.min_distance = float('inf')   #parameter to assign nearest obstacle distance

        # Initialize node
        rospy.init_node('tangent_bug', anonymous=True)

        #Publishing the bot velocities to the /cmd_vel topic
        self.pub = rospy.Publisher('/cmd_vel', gmsg.Twist, queue_size=1)

        #Subscribing to the /scan topic to get the laserscan data of the obstacles in the surroundings
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback_scan)

        #Subscribing to the /odom topic to get the position of the bot
        self.pos_sub = rospy.Subscriber('/odom', Odometry, self.callback_pose)

    def callback_pose(self, data):
        # Gets current position and orientation (Quaternion) of the bot
        self.pos = data.pose.pose.position           
        x_ori = data.pose.pose.orientation.x
        y_ori = data.pose.pose.orientation.y
        z_ori = data.pose.pose.orientation.z
        w_ori = data.pose.pose.orientation.w

        ori = euler_from_quaternion([x_ori, y_ori, z_ori, w_ori])
        self.theta = ori[2]
        # print(self.theta)

    def callback_scan(self, data):
        #Processing the laserscan data to detect surrounding obstacles.
        self.ranges_ = data.ranges
        min_distance = float('inf')        
    
        for i in range(len(data.ranges)):
            #if the distance of the obstacle detected is less than the repulsive force influence range, then it is taken further for exhibiting repulsive force
            distance = data.ranges[i]
            if distance != 0.0 and distance < self.rr:
                d = distance - self.D_obs       #assigning parameter d as distance from safe region around an obstacle 
                if d < 0.0:
                    d = 0.0001                 #if d is negative, asiigning it the minimal positive value to give max repulsion force.
                #the angle of the obstacle is calculated by multiplying the angle increment to the iteration number and adding this value to minimum angle.
                angle = data.angle_min + i * data.angle_increment    
                #adding the if statement below to ensure that only obstacles that obstruct the path of the bot would have their repulsive force influenced on it 
                if angle > (3*(math.pi)/4) and angle < (5*(math.pi)/4):
                    if distance < min_distance:
                        min_distance = distance        #Calculating the distance of nearest obstacle.
                    
                    o_theta = angle + self.theta + math.pi         #converting the obstacle position from robot frame to world frame.
                    #calculating repulsive force of that particular obstacle point 
                    F_rep =  self.K_r * (1 / d - 1 / self.rr) / (d) **2     
                    #adiing the repulsive force vectors of all the obstacles which gives the final resultant repulsive force vector in X and Y direction.
                    self.x_r += F_rep * np.cos(o_theta)
                    self.y_r += F_rep * np.sin(o_theta)
        self.min_distance = min_distance
        print("Min Dist:", min_distance)
        # print("Repulsive force: x_r =", self.x_r, ", y_r =", self.y_r)

    def run(self):
        twist = gmsg.Twist()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Calculating Attraction Force 

            # Calculating the distance and the Euclidian distance between the goal pose coordinates and bot coordinates
            d_x = self.x_goal - self.pos.x
            d_y = self.y_goal - self.pos.y
            dist = np.hypot(d_x, d_y)
            print("bot pose                       ", self.pos.x, self.pos.y)
            F_att = self.K_a * dist
            # calculating Attraction force in X and Y direction
            x_a = F_att * d_x
            y_a = F_att * d_y

            # print("Attraction Force:", x_a,y_a)

            # Take the resultant Repulsive Force
            x_r = self.x_r
            y_r = self.y_r

            # Calculate Final resultant force in X and Y directions
            x_f = x_a - x_r
            y_f = y_a - y_r
            d_f = np.hypot(x_f,y_f)
            # print("resultant force:", x_f, y_f)
            # Initialize the repulsive force vectors to zero
            self.x_r = 0.0
            self.y_r = 0.0
            
            theta_f = np.arctan2(y_f, x_f)                      # calculate angle of the final resultant force vector 
            delta = theta_f - self.theta                        # Calculate the error in bot angle from the resultant force vector.
            delta = np.arctan2(np.sin(delta), np.cos(delta))
            # print("Delta:",delta)

            # Calculate the linear velocity of the bot
            #I have varied proportional gain constants based on the distance from the goal to get more smoother movements and to stop the bot move slower than required when it is in the vicinity of goal point.
            if dist < 0.5:
                v_x = self.kp_l2 * (d_f)
            elif dist > 2.0:
                v_x = self.kp_l1 * (d_f)
            else:
                kp_x = self.kp_l2 + (((self.kp_l2) - self.kp_l1)*(dist - 0.5) / 1.5)      #this is a function i wrote which varies the proportional gain value as a function of distance (for smoother movement of bot when goal point is nearer)
                v_x = kp_x * (d_f)

            # print ("Before clip:",v_x)
            v_x = np.clip(v_x, -1 * self.Vmax, self.Vmax)         #clipping the linear velocity to the max velocity limit.

            # Calculate the angular velocity of the bot
            
            if self.min_distance < 0.3:        
                v_x = 0.025 #self.kp_l3 * (d_f)                            #if distance of nearest obstacle is less than 0.3m in FOV, the velocity gets reduced.
                w_z = self.kp_w_1 * delta
                # v_x = np.clip(v_x, -0.05, 0.05)
            elif self.min_distance == float('inf'):
                w_z = self.kp_w_2 * delta              #if there is no obstacle detected, we assign a new angular proportional gain.
            else:
                w_z = self.kp_w_1 * delta


            
            if dist < 0.25:
                # if the goal point is reached, stop the bot
                print("Reached goal:", self.pos.x, self.pos.y)
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.pub.publish(twist)
                break
            else:
                #if the goal point is not yet reached, publish the calculated linear and angular velocities to /cmd_vel topic
                twist.linear.x = v_x
                # print(v_x)
                twist.angular.z = w_z               
                self.pub.publish(twist)
            rate.sleep()



if __name__ == '__main__':
    try:
        #give the goal point coordinates
        g =  [[4.0,0.0], [-1.0,0.0]] #[[8.0, 0.0], [8.0, 7.0], [11.0, 7.0], [14.0, 5.5], [19.0, 6.25]] #[[7.0, 0.0], [7.0, 4.0], [9.0, 2.0], [15.0, 2.5]] 
        l = len(g)
        i = 0
        for i in range(l):
            
            x_goal = g[i][0]
            y_goal = g[i][1]
        # Run the VelocityController node
            bug_avoid = VelocityController(x_goal, y_goal)
            bug_avoid.run()
    except rospy.ROSInterruptException:
        pass
