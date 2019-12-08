#!/usr/bin/python
import threading
import cv2
from math import atan2, sqrt, pi
import tf
import rospy
from geometry_msgs.msg  import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import pacmanGUI


class RosController(pacmanGUI.MainWindowController):
    def __init__(self):
        rospy.init_node('GUI', anonymous=True)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/webcam/image_raw", Image, self.displayImg)

    def getResourceFolder(self):
        return rospy.get_param("~resourceFilePath", 
                "~/grobots/pacman_ws/src/pacman_config/pacman.ui")
    
    def closeWindow(self):
        self.getMainWindow().close()

    def registerCallbacks(self):
        rospy.on_shutdown(self.closeWindow)

    # UI ros action callbacks
    def toggleEStop(self, active):
        print(active)

    def displayImg(self, img):
        try:
            cvimg = self.bridge.imgmsg_to_cv2(img, "rgb8")
        except CvBridgeError as e:
            print(e)
        self.getMainWindow().displayImageRaw(cvimg)

if __name__ == '__main__':
    rosCon = RosController()
    pacmanGUI.initGUI(rosCon)
    #rospy
    pacmanGUI.runGUI()
    rospy.spin() 
