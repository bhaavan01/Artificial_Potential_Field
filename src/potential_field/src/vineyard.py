#!/usr/bin/env python

import rospy
import sys
import geometry_msgs.msg as gmsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import math
import numpy as np


class VelocityController():
    def __init__(self):
        # User defined parameters

        self.Vmax = 0.2  # Max velocity Limit for the robot
        self.kp_l1 = 0.0005  # Linear Controller proportional gain1
        self.kp_l2 = 0.01  # Linear Controller proportional gain2
        self.kp_w_1 = 1.0   # Angular Controller proportional gain
        self.kp_w_2 = 0.3

        # Position
        self.pos = gmsg.Point()
        #Set the initial positional values of the bot to zero.
        self.pos.x = 0.0
        self.pos.y = 0.0
        self.theta = 0.0

        # Assign the goal coordinates
        # self.x_goal = x_goal
        # self.y_goal = y_goal

        # Attractive and Repulsive force params
        self.K_a = 20     # Attractive force proportional constant
        self.K_a2 = 800
        self.K_r = 2.5      # Repulsive force proportional constant
        self.rr = 2        # Region of influence of the Repulsive force
        self.x_r = 0.0     #Resultant repulsive force vector in X direction
        self.y_r = 0.0     #Resultant repulsive force vector in Y direction
        self.min_distance_r = float('inf')
        self.min_distance_l = float('inf')
        self.x_l = 0.0
        self.y_l = 0.0
        self.F_att_x = 0.0
        self.F_att_y = 0.0
        # Initialize node
        rospy.init_node('tangent_bug', anonymous=True)

        #Publishing the bot velocities to the /cmd_vel topic
        self.pub = rospy.Publisher('/cmd_vel', gmsg.Twist, queue_size=1)

        #Subscribing to the /scan topic to get the laserscan data of the obstacles in the surroundings
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback_scan)

        #Subscribing to the /odom topic to get the position of the bot
        # self.pos_sub = rospy.Subscriber('/odom', Odometry, self.callback_pose)
        self.ori_sub = rospy.Subscriber('/imu', Imu, self.imu_callback_ori)

    def imu_callback_ori(self, data):
        # Gets current position and orientation (Quaternion) of the bot
        # self.pos = data.pose.pose.position
        orientation = data.orientation           
        x_ori = orientation.x
        y_ori = orientation.y
        z_ori = orientation.z
        w_ori = orientation.w

        ori = euler_from_quaternion([x_ori, y_ori, z_ori, w_ori])
        self.theta = ori[2]

    def callback_scan(self, data):
        #Processing the laserscan data to detect surrounding obstacles.
        self.ranges_ = data.ranges
        min_distance_l = float('inf')
        # min_index_l = -1
        min_distance_r = float('inf')
        # min_index_r = -1
    
        for i in range(len(data.ranges)):
            #if the distance of the obstacle detected is less than the repulsive force influence range, then it is taken further for exhibiting repulsive force
            distance = data.ranges[i]
            # if distance < self.rr:          
            # d = distance - 0.4
            # if d < 0.0:
            #     d = 0.0001
            #the angle of the obstacle is calculated by multiplying the angle increment to the iteration number and adding this value to minimum angle.
            angle = data.angle_min + i * data.angle_increment    
            if angle < ((math.pi)/2):
                if distance < min_distance_r:
                    min_distance_r = distance
                    # min_index_r = i
                    o_theta = angle + self.theta                                   #converting the obstacle position from robot frame to world frame.
                # F_rep = self.K_r * ((1 / d) - (1 / self.rr)) / (d)**2                        
                    self.x_r =  (distance * np.cos(o_theta))
                    self.y_r =  (distance * np.sin(o_theta))

            elif angle > (3*(math.pi)/2):
                if distance < min_distance_l:
                    min_distance_l = distance
                    # min_index_l = i
                    o_theta = angle + self.theta                                   #converting the obstacle position from robot frame to world frame.
                # F_rep = self.K_r * ((1 / d) - (1 / self.rr)) / (d)**2                        
                    self.x_l =  (distance * np.cos(o_theta))
                    self.y_l =  (distance * np.sin(o_theta))

        err_x = ((self.x_l + self.x_r)/2)
        err_y = ((self.y_l + self.y_r)/2)
        self.F_att_x = self.K_a2 * err_x
        self.F_att_y = self.K_a2 * err_y
        self.min_distance_l = min_distance_l
        self.min_distance_r = min_distance_r
        print("Min Dist_l:", min_distance_l)
        print("Min Dist_r:", min_distance_r)
        # print("Repulsive force: x_r =", self.x_r, ", y_r =", self.y_r)

    def run(self):
        twist = gmsg.Twist()
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            # Calculating Attraction Force 

            # Calculating the distance and the Euclidian distance between the goal pose coordinates and bot coordinates
            # d_x = self.x_goal - self.pos.x
            # d_y = self.y_goal - self.pos.y
            # dist = np.hypot(d_x, d_y)
            # # print("distance", dist)
            # F_att = self.K_a * dist
            # # calculating Attraction force in X and Y direction
            # x_a = F_att * d_x
            # y_a = F_att * d_y

            # print("Attraction Force:", x_a,y_a)

            # Take the resultant Repulsive Force
            # x_r = self.x_r
            # y_r = self.y_r

            # Calculate Final resultant force in X and Y directions
            x_f = self.F_att_x
            y_f = self.F_att_y
            d_f = np.hypot(x_f,y_f)
            # print("resultant force:", x_f, y_f)
            # print("theta:", self.theta)
            # Initialize the repulsive force vectors to zero
            self.x_r = 0.0
            self.y_r = 0.0
            self.x_l = 0.0
            self.y_l = 0.0
          

            theta_f = np.arctan2(y_f, x_f)                      # calculate angle of the final resultant force vector 
            delta = (theta_f - self.theta)                        # Calculate the error in bot angle from the resultant force vector.
            delta = (np.arctan2(np.sin(delta), np.cos(delta)))*3/4

            # Calculate the linear velocity of the bot
            
            # if dist < 2.5:
            #     v_x = self.kp_l2 * (y_f)
            # else:
            v_x = self.kp_l1 * (y_f)
            # print ("Before clip:",v_x)
            v_x = np.clip(v_x, -1 * self.Vmax, self.Vmax)

            # Calculate the angular velocity of the bot
            # if self.min_distance < 0.3:
            #     w_z = self.kp_w_2 * delta
            #     v_x = 0.05
            # else:
            w_z = self.kp_w_1 * delta
            # print (v_x)
            twist.linear.x = v_x
            twist.angular.z = w_z               
            self.pub.publish(twist)

            # if dist < 0.2:
            #     # if the goal point is reached, stop the bot
            #     print("Reached goal:", self.pos.x, self.pos.y)
            #     twist.linear.x = 0.0
            #     twist.angular.z = 0.0
            #     self.pub.publish(twist)
            #     break
            # else:
            #     #if the goal point is not yet reached, publish the calculated linear and angular velocities to /cmd_vel topic
            #     twist.linear.x = v_x
            #     twist.angular.z = w_z               
            #     self.pub.publish(twist)
            rate.sleep()



if __name__ == '__main__':
    try:
        #give the goal point coordinates
        # g = [[6.5, 9.0]]#, [7.0, 7.5], [7.0, 2.0]]
        # l = len(g)
        # i = 0
        # for i in range(l):
            
        #     x_goal = g[i][0]
        #     y_goal = g[i][1]

        # Run the VelocityController node
            bug_avoid = VelocityController()
            bug_avoid.run()
    except rospy.ROSInterruptException:
        pass
