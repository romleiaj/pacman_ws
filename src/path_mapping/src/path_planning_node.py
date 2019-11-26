#! /usr/bin/python

import rospy
from tf.transformations import euler_from_quaternion
import yaml
import Queue
import cv2
import pdb
import os
import message_filters
import numpy as np
import skimage.graph
from scipy import signal
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from pacman_msgs.msg import PointArray

class PathPlanning():
    def __init__(self, img_in, path_out, h_yaml, odom_in):
        self.pose = None
        self.heading = None
        self.odom_in = odom_in
        self.img_in = img_in
        self.bridge = CvBridge()
        self.path_pub = rospy.Publisher(path_out, PointArray, queue_size=10)    
        self.path_img_pub = rospy.Publisher(path_out + "_img", Image, queue_size=1)    
        self.kernel = np.ones((5,5), np.uint8)
        self.warped_q = Queue.Queue(1)
        self.cost_func = lambda u, v, e, prev_e: e['cost']
        self.fx = self.fy = 1
        self.toMeters = (0.03048 / self.fx) # Conversion from 1/10th feet to meters
        self.cam_offset = 1.01 # Meter offset from camera to bottom of image
        self.ready = True
        with open(h_yaml, 'r') as stream:
            try:
                H = yaml.safe_load(stream)
            except yaml.YAMLError as e:
                print(e)
        self.homography = np.asarray(H['homography'])
        self.output_size = tuple(H['output_shape'])
        self.input_size = tuple(H['input_shape'][0:2])

    def load_odom(self, odom):
        self.pose = odom.pose.pose.position
        o = odom.pose.pose.orientation
        quat = (o.x, o.y, o.z, o.w)
        self.heading = euler_from_quaternion(quat)[2]

    def process_image(self, img_data):
        # If still processing, wait till more recent image
        if not self.ready:
            return

        try:
            raw_cv_img = self.bridge.imgmsg_to_cv2(img_data, desired_encoding="rgb8")
        except CvBridgeError as e:
            rospy.logerr(e)
        rospy.loginfo("Loaded image into queue.")
        # Pulling out green channel (path probabilities)
        mid = int(len(raw_cv_img[0, :, 0])/2.)
        cropped = raw_cv_img[:, mid:-1, :]
        green = cropped[:, :, 1]
        resized_green = cv2.resize(green, self.input_size[::-1]) 
        warped = cv2.warpPerspective(resized_green, self.homography, 
                self.output_size) 
        cv2.imwrite("/home/grobots/Pictures/warped.png", warped)
        self.warped_q.put(warped)
    
    def get_sink(self, img):
        x = 1
        y = 0
        h,w = img.shape
        binarized = (img > 70).astype(np.uint8)
        pad = np.zeros((34, w), dtype=np.uint8)
        pad[:, int(3*w/7):int(4*w/7)] = np.ones((1, int(4*w/7) - int(3*w/7)))
        row_extend = np.append(binarized[:-int(h/20.), :], pad, axis=0)
        new_h, new_w = row_extend.shape
        cv2.imwrite("/home/grobots/Pictures/appended.png", row_extend)
        # Eroding and dilating path clumps
        erosion = cv2.erode(row_extend, self.kernel, iterations = 2)
        dilation = cv2.dilate(erosion, self.kernel, iterations = 2)
        mask = np.zeros_like(dilation)
        # TODO figure out if necessary
        mask = np.pad(mask, (1, 1), 'constant')
        seedPoint = (int(new_w / 2.), new_h - 35)
        dilation[h:,int(3*w/7):int(4*w/7)] = np.ones((1, int(4*w/7) - int(3*w/7)))# * 255
        flooded = cv2.floodFill(dilation, mask, seedPoint, 125)
        flooded = (flooded[1] == 125).astype(np.uint8)# * 255
        #cv2.circle(flooded, seedPoint, 3, (255, 0, 0))
        cv2.imwrite("/home/grobots/Pictures/flooded.png", flooded)
        path_indices = np.nonzero(flooded)
        y_sink = np.min(path_indices[y])
        y_indices = (path_indices[y] == [y_sink])
        x_goal_pts = path_indices[x][y_indices]
        x_goal_regions = self.consecutive(x_goal_pts)
        widest_region = sorted(x_goal_regions, key = len, reverse=True)[0]
        mid_i = int(len(widest_region)/2.)
        x_sink = widest_region[mid_i]
        return (x_sink, y_sink)

    def path_planning(self, warped_img):
        x = 1
        y = 0
        ds_image = cv2.resize(warped_img, (int(self.fx * 
            warped_img.shape[x]), int(warped_img.shape[y] * self.fy)))
        cv2.imwrite("/home/grobots/Pictures/ds_image.png", ds_image)
        costs = (255 - ds_image)
        x_sink, y_sink = self.get_sink(ds_image)
        h, w =  ds_image.shape
        w_2 = int(w/2.)


        rospy.loginfo("Mid point at : (%s, %s)" % (x_sink, y_sink))
        output = np.zeros((h, w, 3), dtype=np.uint8)
        output[:, :, 1] = ds_image

        # Publish estimate path
        cv2.circle(output, (w_2, h-20), 1, (0, 0, 255), thickness=3)
        cv2.circle(output, (x_sink, y_sink), 1, (255, 0, 0), thickness=3)
        path, cost = skimage.graph.route_through_array(costs, start=(h-20, w_2), 
                end=(y_sink, x_sink), fully_connected=True)
        path = np.array(path)
        if len(path) > 40: # Only smooth longer paths
            #path.T[1] = signal.savgol_filter(path.T[1], 11, 3)
            #path.T[0] = signal.savgol_filter(path.T[0], 11, 3)
            b, a = signal.butter(3, 0.05)
            smoothed_path = signal.filtfilt(b, a, path.T[1][20:])
            path.T[1][20:] = [int(dest) for dest in smoothed_path]
            #path.T[1] = signal.medfilt(path.T[1], kernel_size=5)
        
        #contours = cv2.findContours(ds_image, cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)[-2]
        #for contour in contours:
        #    cv2.drawContours(output, contour, -1, (0, 255, 0), 2)
        for loc in path:
            output[loc[y], loc[x], :] = (255, 0, 0)
        try:
            img_msg = self.bridge.cv2_to_imgmsg(output, encoding="rgb8")
        except CvBridgeError as e:
            rospy.logerr(e)
        rospy.logerr(self.heading)
        self.path_img_pub.publish(img_msg)
        
        tx = self.cam_offset
        scaled_pts = [(((w_2 - i) * self.toMeters), ((h - j) * self.toMeters) + tx) 
                for j, i in path]
        
        theta = -self.heading
        # Funky coord system with -y being left and +x forward
        # Flipping axis
        R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), 
            np.cos(theta)]])
        # Rotation happening in meter space
        rotated_list = R.dot(np.array(scaled_pts).T)
        pt_array = PointArray()
        pt_list = []
        for e, n in enumerate(rotated_list[x]):
            pt = Point()
            pt.x = n + self.pose.x
            pt.y = rotated_list[y][e] + self.pose.y
            pt_list.append(pt)
        
        pt_array.points = pt_list
        pt_array.header.stamp = rospy.Time.now()
        self.path_pub.publish(pt_array)

    # https://stackoverflow.com/questions/7352684/how-to-find-the-groups-of-consecutive-elements-from-an-array-in-numpy
    def consecutive(self, data, stepsize=1):
            return np.split(data, np.where(np.diff(data) != stepsize)[0]+1)

    def spin(self):
        img_sub = rospy.Subscriber(self.img_in, Image, self.process_image)
        odom_sub = rospy.Subscriber(self.odom_in, Odometry, self.load_odom)
        rospy.loginfo("Waiting for messages on %s..." % self.img_in)
        
        while not rospy.is_shutdown():
            rospy.sleep(0.01)
            try:
                warped_img = self.warped_q.get_nowait()
                self.ready = False
                self.path_planning(warped_img)
                self.ready = True
            except Queue.Empty:
                pass

        rospy.spin()


def main():
    rospy.init_node("path_planning")
    sub_topic = rospy.get_param("~img_in", default="/webcam/image_segmented")
    pub_topic = rospy.get_param("~path_out", default="/path_points")
    odom_in = rospy.get_param("~odom_in", default="/odom")
    homography_yaml = rospy.get_param("~homography_yaml", 
            default=os.path.expanduser("~/pacman_ws/src/utility_scripts/scripts/homography.yaml"))
    pp = PathPlanning(sub_topic, pub_topic, homography_yaml, odom_in)
    pp.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
        pass

