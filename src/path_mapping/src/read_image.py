#!/usr/bin/python3

import numpy as np
import rospy 
import cv2
import utm
import pdb
import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt 
import yaml
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
class Reader():
    def __init__(self):
        self.first_gps = None
        self.latest_image = None
        self.heading = None
        self.adjusted = None
        self.height = 4800
        self.width = 6400
        self.canvas = np.zeros((self.height,self.width,3), np.uint8)

        rospy.init_node('bla',anonymous=True)
        homography_filename = rospy.get_param("~homography")
        #sub_vis = rospy.Subscriber('/webcam/image_raw',Image,self.vis_callback)
        sub_vis = rospy.Subscriber('/webcam/image_segmented',Image,self.vis_callback)
        sub_gps = rospy.Subscriber('/piksi/enu/pose_best_fix',Odometry, self.gps_callback)
        with open(homography_filename) as f:
            self.homography = np.array(yaml.load(f))

        rospy.spin()

    def vis_callback(self, img ):
        #Wprint(img.data)
        try:
            im = bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
            cv2.imwrite("segmented.png", im)
            self.latest_image = im
            print(self.latest_image.shape)
            size = 100
            scale = 20
            if self.adjusted is not None:
                x = int(self.adjusted[0] * scale + self.width / 2.0)
                y = int(self.adjusted[1] * scale + self.height / 2.0)
                warped = cv2.warpPerspective(self.latest_image[:,int(self.latest_image.shape[1]/2.0):, :], self.homography, (size, size))
                heading_angle =np.rad2deg( np.arctan(self.heading[1] / self.heading[0] ) )
                print(heading_angle)
                rotated = self.rotateImage(warped, heading_angle)
                #resized = cv2.resize(self.latest_image, (size, size))
                print(rotated.shape)
                self.canvas[y:y+size, x:x+size, :] = rotated
                #self.canvas[y:y+size, x:x+size, :] = resized
                cv2.imwrite("canvas.png", self.canvas)
        except CvBridgeError as e:
            rospy.logerr(e)
   
    def rotateImage(self, image, angle):
        image_center = tuple(np.array(image.shape[1::-1]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
        result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
        return result

    def gps_callback(self, data ):
        position = data.pose.pose.position
        #print(data)
        if self.first_gps is None:
            self.first_gps = np.asarray([position.x, position.y])
        new_adjusted = np.asarray([position.x, position.y] - self.first_gps)
        if self.adjusted is not None:
            self.heading = new_adjusted - self.adjusted 
        self.adjusted = new_adjusted
            
        plt.scatter(self.adjusted[0], self.adjusted[1])
        #print(adjusted)
        #utm.from_latlon(position.x, position.y)


reader = Reader()
