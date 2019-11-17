#!/usr/bin/python
import threading
import rospy
from geometry_msgs.msg  import Twist, Vector3
from nav_msgs.msg import Odometry
from math import atan2, sqrt, pi
# from pacman_msgs.msg import PointArray

import tf



class RosController():
    def __init__(self, window):
        self.window = window
        thread = threading.Thread(self.rosInit)
        thread.start()

    def rosInit(self):
        rospy.init_node('GUI', anonymous=True)
        rospy.spin()
        self.window.close()

    def killNode(self):
        rospy.signal_shutdown("User closed window")
