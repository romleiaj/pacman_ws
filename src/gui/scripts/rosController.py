#!/usr/bin/python
import threading
import rospy
from geometry_msgs.msg  import Twist, Vector3
from nav_msgs.msg import Odometry
from math import atan2, sqrt, pi
import pacmanGUI

import tf



class RosController(pacmanGUI.MainWindowController):
    def __init__(self):
        rospy.init_node('GUI', anonymous=True)

    def getResourceFolder(self):
        return rospy.get_param("~resourceFilePath", "~/grobots/pacman_ws/src/pacman_config/pacman.ui")
    
    def closeWindow(self):
        self.getMainWindow().close()

    def registerCallbacks(self):
        rospy.on_shutdown(self.closeWindow)

    # UI ros action callbacks
    def toggleEStop(self, active):
        print(active)

    

if __name__ == '__main__':
    rosCon = RosController()
    pacmanGUI.initGUI(rosCon)
    rospy
    pacmanGUI.runGUI()
    
