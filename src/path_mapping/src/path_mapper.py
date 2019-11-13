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
from scipy import ndimage
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
        self.height = 4000
        self.width = 4000 
        self.canvas = np.zeros((self.height,self.width,3), np.uint8)

        rospy.init_node('bla',anonymous=True)
        homography_filename = rospy.get_param("~homography", default="/home/grobots/pacman_ws/src/pacman_config/homography.yaml")
        print("the homography file name is {}".format(homography_filename))
        sub_vis = rospy.Subscriber('/webcam/image_segmented',Image,self.vis_callback)
        sub_gps = rospy.Subscriber('/encoders/odom',Odometry, self.gps_callback)
        with open(homography_filename) as f:
           info =  yaml.load(f)
           print(info)
           self.homography = np.asarray(info["homography"])
           input_shape = info["input_shape"]
           self.input_shape = (input_shape[1], input_shape[0])
           output_shape = info["output_shape"]
           self.output_shape = (output_shape[1], output_shape[0])
        print("initialized node")

        rospy.spin()

    def vis_callback(self, img, scale=30.48): # centimeters in a foot
        #Wprint(img.data)
        try:
            im = bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
            cv2.imwrite("segmented.png", im)
            self.latest_image = im
            print(self.latest_image.shape)
            if self.adjusted is not None:
                print("adjusted {}".format(self.adjusted))
                x = int(self.adjusted[0] * scale + self.width / 2.0)
                y = int(self.adjusted[1] * scale + self.height / 2.0)
                segmentation_half = self.latest_image[:,int(self.latest_image.shape[1]/2.0):, :]
                resized = cv2.resize(segmentation_half, self.input_shape)
                cv2.imwrite("resized.png", resized)
                warped = cv2.warpPerspective(resized, self.homography, self.output_shape)
                padded_warped = np.concatenate((warped, np.zeros_like(warped, dtype=np.uint8)), axis=0) # make the robot at the center # TODO change this to zeros_like

                cv2.imwrite("patted_warped.png", padded_warped)
                heading_angle =np.rad2deg( np.arctan(self.heading[1] / self.heading[0] ) )
                heading_angle += 180 # Trying to determine the correct convention
                print(heading_angle)
                rotated = ndimage.rotate(padded_warped, heading_angle)
                cv2.imwrite("rotated_warped.png", rotated)
                #resized = cv2.resize(self.latest_image, (size, size))
                print("rotated shape {}".format(rotated.shape))
                #self.canvas[y:y+rotated.shape[0], x:x+rotated.shape[1], :] = np.ones_like(rotated).astype(np.uint8) * 255
                incremented = np.minimum( rotated + self.canvas[y:y+rotated.shape[0], x:x+rotated.shape[1], :], 255)
                self.canvas[y:y+rotated.shape[0], x:x+rotated.shape[1], :] = incremented
                cv2.imwrite("/home/grobots/pacman_ws/src/path_mapping/src/canvas.png", self.canvas)
        except CvBridgeError as e:
            rospy.logerr(e)
   
    def rotateImage(self, image, angle):
        image_center = tuple(np.array(image.shape[1::-1]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
        result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
        return result

    def gps_callback(self, data ):
        position = data.pose.pose.position
        if self.first_gps is None:
            self.first_gps = np.asarray([position.x, position.y])
        new_adjusted = np.asarray([position.x, position.y] - self.first_gps)
        if self.adjusted is not None:
            self.heading = new_adjusted - self.adjusted 
        self.adjusted = new_adjusted
            
        plt.scatter(self.adjusted[0], self.adjusted[1])
        if np.random.randint(100) == 0:
            print("saving")
            plt.savefig("/home/grobots/pacman_ws/src/utility_scripts/scripts/gps.png")

        #print(adjusted)
        #utm.from_latlon(position.x, position.y)


reader = Reader()
