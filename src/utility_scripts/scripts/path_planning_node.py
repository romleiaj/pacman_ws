#! /usr/bin/python

import rospy
import yaml
import Queue
import cv2
import pdb
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from pacman_msgs.msg import PointArray
from dijkstar import Graph, find_path

class PathPlanning():
    def __init__(self, img_in, path_out, h_yaml):
        self.img_in = img_in
        self.bridge = CvBridge()
        self.path_pub = rospy.Publisher(path_out, PointArray, queue_size=10)    
        self.kernel = np.ones((5,5), np.uint8)
        self.flooded_Q = Queue.Queue(1)
        self.cost_func = lambda u, v, e, prev_e: e['cost']
        self.fx = self.fy = .1
        self.ready = True
        with open(h_yaml, 'r') as stream:
            try:
                H = yaml.safe_load(stream)
            except yaml.YAMLError as e:
                print(e)
        self.homography = np.asarray(H['homography'])
        self.output_size = tuple(H['output_shape'])
        self.input_size = tuple(H['input_shape'][0:2])

    def process_image(self, data):
        # If still processing, wait till more recent image
        if not self.ready:
            return

        try:
            raw_cv_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="rgb8")
        except CvBridgeError as e:
            rospy.logerr(e)
        # Pulling out green channel (path probabilities)
        green = raw_cv_img[:, :, 1]
        resized_green = cv2.resize(green, self.input_size) 
        warped = cv2.warpPerspective(resized_green, self.homography, 
                self.output_size) 
        cv2.imwrite("/home/adam/Pictures/warped.png", warped)
        # Thresholding path probability
        binarized = (warped > 100).astype(np.uint8) * 255
        # Eroding and dilating path clumps
        erosion = cv2.erode(binarized, self.kernel, iterations = 1)
        dilation = cv2.dilate(erosion, self.kernel, iterations = 1)
        mask = np.zeros_like(dilation)
        # TODO figure out if necessary
        mask = np.pad(mask, (1,1), 'constant')
        seedPoint = (int(warped.shape[0] / 2), warped.shape[1] - 1)
        flooded = cv2.floodFill(dilation, mask, seedPoint, 125)
        flooded = (flooded[1] == 125).astype(np.uint8) * 255
        
        cv2.imwrite("/home/adam/Pictures/flood.png", flooded)
        
        self.flooded_Q.put(flooded)
        return 0

    def path_planning(self, flooded_img):
        # TODO: FIgure out dynamically
        y_sink = 15
        x_sink = 99
        ds_image = cv2.resize(flooded_img, (int(self.fx * 
            flooded_img.shape[1]), int(flooded_img.shape[0] * self.fy)))
        cv2.imwrite("/home/adam/Pictures/ds_image.png", ds_image)
        h, w =  ds_image.shape
        output = np.zeros((h, w, 3), dtype=np.uint8)
        # TODO: Change 255 to 1 if not displayed
        graph = self.image2graph(ds_image, 255)
        source = w * (h - 1) + int(w/2)
        sink   = w * y_sink + x_sink
        path = find_path(graph, source, sink, cost_func=self.cost_func)
        path_elems = path[0]
        np.save("path.npy", path_elems)
        path_elems = np.load("path.npy")
        contours = cv2.findContours(flooded_img, cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)[-2]
        for contour in contours:
            cv2.drawContours(output, contour, -1, (0, 255, 0), 3)
        for path_elem in path_elems:
            loc = self.ind2ij(path_elem, w)
            output[loc[0], loc[1], :] = (255, 0, 0)

        cv2.imwrite("/home/adam/Pictures/planned_path.png", output)
        # publish points


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

    def ind2ij(self, ind, width):
        return (int(np.floor(ind / width)), int(ind % width))

    def spin(self):
        rospy.Subscriber(self.img_in, Image, self.process_image)
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
    homography_yaml = rospy.get_param("~homography_yaml", 
            default="/home/adam/pacman_ws/src/utility_scripts/scripts/homography.yaml")
    pp = PathPlanning(sub_topic, pub_topic, homography_yaml)
    pp.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
        pass

