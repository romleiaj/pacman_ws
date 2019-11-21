#! /usr/bin/python

import rospy
import pdb
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib
matplotlib.use('Agg') # because it's a server

import matplotlib.pyplot as plt

class AbsoluteOdom():
    def __init__(self, rate=1):
        self.rate = rate
        rospy.init_node("OdometryAccumulator")
        self.pub = rospy.Publisher("/absolute_odom", Odometry,queue_size=10)
        self.subscriber = rospy.Subscriber("/encoders/odom", Odometry, self.read_odom)
        self.input_odom = None
        self.output_odom = Odometry()
        rospy.spin()

    def read_odom(self, msg):
        self.output_odom.pose.pose.position.x += msg.pose.pose.position.x
        self.output_odom.pose.pose.position.y += msg.pose.pose.position.y
        self.output_odom.pose.pose.position.z += msg.pose.pose.position.z

        self.output_odom.pose.pose.orientation.x += msg.pose.pose.orientation.x
        self.output_odom.pose.pose.orientation.y += msg.pose.pose.orientation.y
        self.output_odom.pose.pose.orientation.z += msg.pose.pose.orientation.z
        self.output_odom.pose.pose.orientation.w += msg.pose.pose.orientation.w
        self.pub.publish(self.output_odom)
        plt.scatter(msg.pose.pose.position.x, msg.pose.pose.position.y)
        if np.random.randint(100) == 0:
            print("saving")
            plt.savefig("/home/grobots/pacman_ws/src/utility_scripts/scripts/gps.png")

a = AbsoluteOdom()




