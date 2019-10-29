#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from math import pi
import tf  # transform?
import pdb
import numpy as np

class OdomReader:
    def __init__(self, odom_topic):
        
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.last_x = 0
        self.last_y = 0
        self.last_yaw = 0
        self.last_pitch = 0
        self.last_roll = 0
        self.last_angular_z = 0
        
        
    def odom_callback(self, message):
        """Successfully reading location"""
        self.last_x = message.pose.pose.position.x
        self.last_y = message.pose.pose.position.y
        self.last_angular_z = message.twist.twist.angular.z
        self.last_angular_y = message.twist.twist.angular.y
        self.last_angular_x = message.twist.twist.angular.x
        
        
        orient = message.pose.pose.orientation
        [self.last_roll, self.last_pitch, self.last_yaw] = tf.transformations.euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])        
    
    def get_x(self):
        return self.last_x
        
    def get_y(self):
        return self.last_y
        
    def get_angular_z(self):
        return self.last_angular_z
        
    def get_roll(self):
        return self.last_roll
        
    def get_pitch(self):
        return self.last_pitch
    
    def get_yaw(self):
        return self.last_yaw/(2*pi)*360 #returns yaw angle (rotation about z) in degrees
