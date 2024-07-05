#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class TurtleBotObstacleAvoidance:
    def __init__(self):
        rospy.init_node('turtlebot_obstacle_avoidance', anonymous=False)

        # Publisher to control the robot's velocity
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscriber to the LaserScan topic
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        self.move_cmd = Twist()

        self.rate = rospy.Rate(10)

    def scan_callback(self, data):

        min_distance = float('inf')
        min_index = -1

        for i, distance in enumerate(data.ranges):
            if distance < min_distance:
                min_distance = distance
                min_index = i
        
        #angle of the nearest obstacle
        angle_min = data.angle_min
        angle_increment = data.angle_increment
        obstacle_angle = angle_min + min_index * angle_increment
        
        rospy.loginfo(f"Nearest obstacle at distance: {min_distance}, angle: {obstacle_angle}")

        # Avoid the obstacle
        self.avoid_obstacle(min_distance, obstacle_angle)
        
    def avoid_obstacle(self, distance, angle):
        # If the obstacle is too close, move in the opposite direction
        if distance < 0.2:  # Threshold distance
            if angle < 2.8:
                # Obstacle is to the left, turn right
                self.move_cmd.angular.z = -0.5
                self.move_cmd.linear.x = 0.0
            elif angle > 3.5:
                # Obstacle is to the right, turn left
                self.move_cmd.angular.z = 0.5
                self.move_cmd.linear.x = 0.0
            else:
                self.move_cmd.linear.x = 0.2
                self.move_cmd.angular.z = 0.0
            
        else:

            self.move_cmd.angular.z = 0.0
            self.move_cmd.linear.x = 0.2

        self.cmd_vel_pub.publish(self.move_cmd)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        bot = TurtleBotObstacleAvoidance()
        bot.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")