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
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from pacman_msgs.msg import PointArray
from dijkstar import Graph, find_path

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
        self.flooded_Q = Queue.Queue(1)
        self.cost_func = lambda u, v, e, prev_e: e['cost']
        self.fx = self.fy = 1
        self.toMeters = 0.03048 # Conversion from 1/10th feet to meters
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

    def process_image(self, img_data, odom):
        # If still processing, wait till more recent image
        if not self.ready:
            return

        try:
            raw_cv_img = self.bridge.imgmsg_to_cv2(img_data, desired_encoding="rgb8")
        except CvBridgeError as e:
            rospy.logerr(e)
        # Pulling out green channel (path probabilities)
        mid = int(len(raw_cv_img[0, :, 0])/2.)
        cropped = raw_cv_img[:, mid:-1, :]
        green = cropped[:, :, 1]
        resized_green = cv2.resize(green, self.input_size[::-1]) 
        cv2.imwrite("/home/grobots/Pictures/resized.png", resized_green)
        warped = cv2.warpPerspective(resized_green, self.homography, 
                self.output_size) 
        cv2.imwrite("/home/grobots/Pictures/warped.png", warped)
        #cv2.imwrite("/home/adam/Pictures/warped.png", warped)
        # Thresholding path probability
        binarized = (warped > 100).astype(np.uint8) * 255
        # Eroding and dilating path clumps
        erosion = cv2.erode(binarized, self.kernel, iterations = 2)
        dilation = cv2.dilate(erosion, self.kernel, iterations = 2)
        mask = np.zeros_like(dilation)
        # TODO figure out if necessary
        mask = np.pad(mask, (1,1), 'constant')
        seedPoint = (int(warped.shape[0] / 2), warped.shape[1] - 1)
        flooded = cv2.floodFill(dilation, mask, seedPoint, 125)
        flooded = (flooded[1] == 125).astype(np.uint8) * 255
        
        #cv2.imwrite("/home/adam/Pictures/flood.png", flooded)
        self.pose = odom.pose.pose.position
        o = odom.pose.pose.orientation
        quat = (o.x, o.y, o.z, o.w)
        self.heading = euler_from_quaternion(quat)[2]
        self.flooded_Q.put(flooded)
        return 0

    def path_planning(self, flooded_img):
        x = 1
        y = 0
        ds_image = cv2.resize(flooded_img, (int(self.fx * 
            flooded_img.shape[x]), int(flooded_img.shape[y] * self.fy)))
        cv2.imwrite("/home/grobots/Pictures/ds_image.png", ds_image)
        h, w =  ds_image.shape
        w_2 = int(w/2.)

        path_indices = np.nonzero(ds_image)
        y_sink = np.min(path_indices[y])
        y_indices = (path_indices[y] == [y_sink])
        x_goal_pts = path_indices[x][y_indices]
        x_goal_regions = self.consecutive(x_goal_pts)
        widest_region = sorted(x_goal_regions, key = len, reverse=True)[0]
        mid_i = int(len(widest_region)/2.)
        x_sink = widest_region[mid_i]

        rospy.loginfo("Mid point at : (%s, %s)" % (x_sink, y_sink))
        output = np.zeros((h, w, 3), dtype=np.uint8)
        output[path_indices] = (255, 255, 255)
        # TODO: Change 255 to 1 if not displayed
        graph = self.image2graph(ds_image, 255)
        source = w * (h - 1) + w_2
        sink   = w * y_sink + x_sink

        #hueristic_func = create_hueristic_func(1,3)

        # Publish estimate path
        cv2.circle(output, (w_2, h-1), 1, (0, 0, 255), thickness=5)
        cv2.circle(output, (x_sink, y_sink), 1, (255, 0, 0), thickness=5)
        path = find_path(graph, source, sink, cost_func=self.cost_func)
        path_elems = path[0]
        contours = cv2.findContours(flooded_img, cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)[-2]
        for contour in contours:
            cv2.drawContours(output, contour, -1, (0, 255, 0), 2)
        for path_elem in path_elems:
            loc = self.ind2ij(path_elem, w)
            output[loc[y], loc[x], :] = (255, 0, 0)
        try:
            img_msg = self.bridge.cv2_to_imgmsg(output, encoding="rgb8")
        except CvBridgeError as e:
            rospy.logerr(e)
        self.path_img_pub.publish(img_msg)
        
        theta = self.heading
        # Funky coord system with -y being left and +x forward
        tx = self.pose.y
        ty = self.pose.x + self.cam_offset
        R = np.array([[np.cos(theta), -np.sin(theta), tx], [np.sin(theta), 
            np.cos(theta), ty], [0, 0, 1]])
        pts = [self.ind2ij(elem, w) for elem in path_elems]
        scaled_pts = [(((i - w_2) * self.toMeters), ((h - j) * self.toMeters), 
            k) for j, i, k in pts]
        # Rotation happening in meter space
        rotated_list = np.matmul(R, np.array(scaled_pts).T)

        pt_array = PointArray()
        pt_list = []
        for mpt in rotated_list.T:
            pt = Point()
            pt.x = mpt[x]
            pt.y = -mpt[y]
            pt_list.append(pt)
        
        pt_array.points = pt_list
        pt_array.header.stamp = rospy.Time.now()
        self.path_pub.publish(pt_array)

    def create_hueristic_func(goal_i, goal_y):
        def hueristic(u, v, edge, prev_edge):
            return np.linalg.norm(np.asarray([goal_x, goal_y]) - np.asarray(edge[1:3]))
        return hueristic


    def image2graph(self, img, true_val = 255, debug=False):
        """
        convert a binary representation to graphical representation
        """
        graph = Graph()
        num_rows = img.shape[0]
        num_cols = img.shape[1]
        img = np.pad(img, (0, 1), 'constant')
        for i in range(num_rows):
            for j in range(num_cols):
                ind_current = i * num_cols + j
                ind_down    = (i + 1) * num_cols + j
                ind_right   = (i * num_cols) + j + 1
                ind_right_down = (i + 1) * num_cols + j + 1
     
                current    = img[i, j]
                down       = img[i + 1, j]
                right      = img[i, j + 1]
                right_down = img[i + 1, j + 1]
    
                if current == true_val and down == true_val:
                    if debug:
                        print("down: {} to {}".format(ind_current, ind_down))
                    graph.add_edge(ind_current, ind_down, {'cost' : 1})
                    graph.add_edge(ind_down, ind_current, {'cost' : 1})
    
                if current == true_val and right == true_val:
                    if debug:
                        print("right: {} to {}".format(ind_current, ind_right))
                    graph.add_edge(ind_current, ind_right, {'cost' : 1})
                    graph.add_edge(ind_right, ind_current, {'cost' : 1}) # make it bidirectional
    
                if current == true_val and right_down == true_val:
                    if debug:
                        print("right down: {} to {}".format(ind_current, ind_right_down))
                    graph.add_edge(ind_current, ind_right_down, {'cost' : np.sqrt(2)})
                    graph.add_edge(ind_right_down, ind_current, {'cost' : np.sqrt(2)})
    
                if right == true_val and down == true_val:
                    if debug:
                        print("left up: {} to {}".format(ind_right, ind_down))
                    graph.add_edge(ind_right, ind_down, {'cost' : np.sqrt(2)})
                    graph.add_edge(ind_down, ind_right, {'cost' : np.sqrt(2)}) 
                    # make it bidirectional
        print(img.shape)
        return graph

    # https://stackoverflow.com/questions/7352684/how-to-find-the-groups-of-consecutive-elements-from-an-array-in-numpy
    def consecutive(self, data, stepsize=1):
            return np.split(data, np.where(np.diff(data) != stepsize)[0]+1)

    def ind2ij(self, ind, width):
        return (int(np.floor(ind / width)), int(ind % width), 0)

    def spin(self):
        img_sub = message_filters.Subscriber(self.img_in, Image)
        odom_sub = message_filters.Subscriber(self.odom_in, Odometry)
        time_sync = message_filters.ApproximateTimeSynchronizer([img_sub, 
            odom_sub], queue_size=60, slop=0.1)
        time_sync.registerCallback(self.process_image)
        rospy.loginfo("Waiting for messages on %s..." % self.img_in)
        
        while not rospy.is_shutdown():
            rospy.sleep(0.01)
            try:
                flooded_img = self.flooded_Q.get_nowait()
                self.ready = False
                self.path_planning(flooded_img)
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

