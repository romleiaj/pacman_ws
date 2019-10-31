#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist, Vector3
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt,pi
from turtlesim.msg import Pose

import numpy as np
import tf
import pdb
import Queue


class PointFilter():

    def __init__(self):
        #Creating node,publisher and subscriber
        rospy.init_node('PointFilter_node', anonymous=True)
        self.point_publisher = rospy.Publisher(filtered_points_topic, tuple, queue_size=100) # Size of point bursts
        self.point_subscriber = rospy.Subscriber(unfiltered_points_topic, tuple, self.point_queue_callback)
        self.queue = []
        self.rate = rospy.Rate(1) # Rate to publish new points, ideally once per segmentation
        
    def point_queue_callback(self, tuple point): #FIFO Queue
        self.queue.insert(0, point)
        
    def get_next_point(self):
        if len(self.queue) < 1: # No points in queue
            print("no points in queue")
            raise Queue.Empty
        else:
            return self.queue.pop()
        
    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance

    def filter(self):
        pose = self.pose.pose.position
        goal_point = self.get_next_point()
        #goal_point = np.asarray(goal_point) #I dont think this is needed
        goal_pose.x = goal_point[0] # x value
        goal_pose.y = goal_point[1] # y value
        elements_after = 1   # How many elements after

        while sqrt(pow((goal_pose.x - pose.x), 2) + pow((goal_pose.y - pose.y), 2)) >= distance_tolerance:

            #Porportional Controller
            if len(self.queue) > 1:

                
            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z =0
        self.point_publisher.publish(tuple)

        rospy.spin()
    

if __name__ == '__main__':
    try:
        #Testing our function
        robot = PathNavigation()
        cmd_vel_topic = rospy.get_param('~cmd_vel_topic', 'pacman_equiv/cmd_vel')
        odom_topic = rospy.get_param('~odom_topic', 'pacman_equiv/odom')
        print(cmd_vel_topic)
        
        robot.move2point()
        while  true:
            # while len(robot.queue) > 0:
            #     robot.mov
                e2point()

    except rospy.ROSInterruptException: pass
