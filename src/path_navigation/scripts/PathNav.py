#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist, Vector3
from nav_msgs.msg import Odometry
from math import atan2, sqrt, pi, cos, copysign
from pacman_msgs.msg import PointArray

import numpy as np
import tf
import pdb
import Queue
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class PathNavigation():

    def __init__(self):
        #Creating node,publisher and subscriber
        self.velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.odom_subscriber = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        self.odom = Odometry()
        self.point_subscriber = rospy.Subscriber("/path_points", PointArray, 
                self.point_queue_callback)
        self.queue = []
        self.rate = rospy.Rate(30)
        self.stopped = True
        self.weighted_path = None
        self.fig = plt.figure() 
        self.ax = self.fig.add_subplot(1, 1, 1)
        # User specified tolerance and gains
        self.distance_tolerance = 0.1
        self.lin_vel = 0.3 #.2 worked
        self.Kp_w = .4

    #Callback function implementing the odom value received
    def odom_callback(self, data):
        self.odom = data
        
    def point_queue_callback(self, msg): #FIFO Queue
        array = msg.points
        np_pts = np.array([[pt.x, pt.y, pt.z] for pt in array]) # convert to list
        self.ax.plot(np_pts[:, 0], np_pts[:, 1], 'r--')
        if self.weighted_path is None or len(self.weighted_path) == 1:
            self.weighted_path = np_pts
        else:
            for j, point in enumerate(np_pts):
                dist_2 = np.sum((self.weighted_path - point)**2, axis=1)
                closeI = np.argmin(dist_2)
                x0 = self.weighted_path[closeI][0]
                y0 = self.weighted_path[closeI][1]
                x1 = point[0]
                y1 = point[1]
                new_pt = (x0 + (x1 - x0)/2., y0 + (y1 - y0)/2., point[2])
                np_pts[j] = new_pt
            self.weighted_path = np_pts
        self.ax.plot(np_pts[:, 0], np_pts[:, 1], 'g--')
        self.ax.axis('equal')
        self.fig.savefig("/home/grobots/figure.png")
        self.weighted_path = np.array([[pt.x, pt.y, pt.z] for pt in array]) # convert to list
        #print("MADE IT")
        #return
            
        dimension = self.weighted_path.shape
        distance_to = 0
        distance_to_prev = 0
        mid_pt = int(len(self.weighted_path)/2.)
        i_valid = False

        if dimension[0] > 1: # Point Filtering
            for i, point in enumerate(self.weighted_path[::-1]):
                #self.queue.insert(0, point)
                distance_to_prev = distance_to
                distance_to = self.get_distance(point[0], point[1])
                rospy.loginfo(distance_to)
                if i >= mid_pt: # valid checking
                    if (distance_to - distance_to_prev < 0) and (distance_to <= self.distance_tolerance):
                        # closest to robot position
                        i_valid = True
                        break
                    # If i is larger than half of the array segmentation can't keep up with speed
            index_offset = 0 
            # Added to account for latency (will be affected by point density in line)
            index = len(self.weighted_path) - i
            filtered_points = self.weighted_path[index+index_offset:]
            print("Split at: " + str(i) + "/" + str(len(self.weighted_path)))
        else:
            filtered_points = self.weighted_path            
            i_valid = True
        if i_valid is True:
            self.queue = [] #TODO: Clear queue
            for point in filtered_points:
                self.queue.insert(0, point)
        else:
            self.queue = [] #TODO: Clear queue
            rospy.logerr("No valid points were found within tolerance of segmentation fps.")
        print("Insert Q Now: " + str(len(self.queue)))
        
    def get_next_point(self):
        if len(self.queue) < 1: # No points in queue
            print("no points in queue")
            raise Queue.Empty
        else:
            print("Popped Q Now: " + str(len(self.queue)))
            return self.queue.pop()
        
    def get_distance(self, goal_x, goal_y):
        pose = self.odom.pose.pose.position
        #rospy.logerr("(%s, %s) | (%s, %s)" % (goal_x, goal_y, pose.x, pose.y))
        distance = sqrt(pow((goal_x - pose.x), 2) + pow((goal_y - pose.y), 2))
        #print("Distance: " + str(distance))
        return distance

    def move2point(self):
        try:
            goal_point = self.get_next_point()
            rospy.logwarn(goal_point)
        except Queue.Empty:
            return
        #goal_point = np.asarray(goal_point) #I dont think this is needed
        vel_msg = Twist()
        goal_x = goal_point[0]
        goal_y = goal_point[1]
        initial_distance = self.get_distance(goal_x, goal_y)
        while self.get_distance(goal_x, goal_y) >= self.distance_tolerance\
                and not rospy.is_shutdown():
            orient = self.odom.pose.pose.orientation
            pose = self.odom.pose.pose.position
            [roll, pitch, yaw] = tf.transformations.euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
            theta = atan2(goal_y - pose.y, goal_x - pose.x)
            
            #Porportional Controller
            
            turn_angle = pi/2
            turn_factor = (turn_angle - abs(theta - yaw)) / turn_angle
            turn_angle = 1
            turn_factor = ((turn_angle - abs(cos(theta) - 
                    cos(yaw))) / turn_angle)
            if turn_factor < 0:
                rospy.logwarn("No forward velocity")
                turn_factor = 0

            distance_factor = (initial_distance - self.get_distance(goal_x, goal_y)) / initial_distance # More zero in the begining and becomes 1 towards end
            if distance_factor < 0.1:
                distance_factor = 0.1
            elif distance_factor > 1:
                distance_factor = 1
            vel = 0
            if self.stopped and len(self.queue) > 0:
                vel = self.lin_vel * turn_factor * distance_factor
                print("Ramping up.")
                self.stopped = False
            elif not self.stopped and len(self.queue) == 0:
                vel = self.lin_vel * turn_factor * (1-distance_factor)
            else:
                vel = self.lin_vel * turn_factor

            if vel > self.lin_vel:
                vel = self.lin_vel
            elif vel < -self.lin_vel:
                vel = -self.lin_vel
            vel_msg.linear.x = vel
            #angular velocity in the z-axis: Add Differential
            delta_theta = theta - yaw
            if delta_theta > pi:
                delta_theta = (delta_theta - 2*pi)
            elif delta_theta < -pi:
                delta_theta = (delta_theta + 2*pi)
            vel_msg.angular.z = (self.Kp_w * sqrt(abs(delta_theta)) * 
                copysign(1, delta_theta))
                
            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        
        if len(self.queue) == 0:
            self.stopped = True
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
        else:
            self.stopped = False
            
if __name__ == '__main__':
    try:
        #Testing our function
        rospy.init_node('PathNav', anonymous=True)
        cmd_vel_topic = rospy.get_param('~cmd_vel_topic', 'pacman_simplified/cmd_vel')
        odom_topic = rospy.get_param('~odom_topic', 'pacman_simplified/odom')
        robot = PathNavigation()
        print(cmd_vel_topic)
        
        while not rospy.is_shutdown(): #len(robot.queue) > 0:
            if len(robot.queue) > 0:
                robot.move2point()
        
        rospy.spin()
                
        #raise SystemExit

        robot.point_queue_callback([(0.755172, -0.013588)])
        robot.point_queue_callback([(0.755172, -0.013588)])
        robot.point_queue_callback([(2.012774, -0.070005)])
        robot.point_queue_callback([(4.350890, -0.339451)])
        robot.point_queue_callback([(4.098817, -1.434626)])
        robot.point_queue_callback([(3.490398, -3.885729)])
        robot.point_queue_callback([(3.420172, -5.579248)])
        robot.point_queue_callback([(3.863448, -7.039472)])
        robot.point_queue_callback([(4.524018, -8.021641)])
        robot.point_queue_callback([(5.680047, -8.769141)])
        robot.point_queue_callback([(7.505334, -8.786546)])
        robot.point_queue_callback([(8.774353, -7.700043)])
        robot.point_queue_callback([(9.617440, -6.335436)])
        robot.point_queue_callback([(9.878201, -5.057729)])
        robot.point_queue_callback([(9.678280, -3.536668)])
        robot.point_queue_callback([(9.000335, -2.467563)])
        robot.point_queue_callback([(7.696537, -1.350658)])
        robot.point_queue_callback([(6.001646, -0.672696)])
        robot.point_queue_callback([(4.332815, -0.290254)])
        robot.point_queue_callback([(4.515331, 0.804919)])
        robot.point_queue_callback([(4.715252, 2.282535)])
        robot.point_queue_callback([(4.567483, 3.690624)])
        robot.point_queue_callback([(3.950347, 4.490277)])
        robot.point_queue_callback([(2.837802, 4.942240)])
        robot.point_queue_callback([(0.491005, 5.107382)])
        robot.point_queue_callback([(-2.142627, 5.950500)])
        robot.point_queue_callback([(-3.194343, 6.611070)])
        robot.point_queue_callback([(-4.246068, 7.488265)])
        robot.point_queue_callback([(-6.106102, 9.096958)])
        robot.point_queue_callback([(-3.072665, 6.524170)])
        robot.point_queue_callback([(-1.421202, 5.724518)])
        robot.point_queue_callback([(-0.395565, 5.011765)])
        robot.point_queue_callback([(-2.429456, 4.690167)])
        robot.point_queue_callback([(-4.585040, 4.046970)])
        robot.point_queue_callback([(-6.914442, 3.021342)])
        robot.point_queue_callback([(-9.209095, 1.309050)])
        robot.point_queue_callback([(-10.425963, -0.090338)])
        robot.point_queue_callback([(-11.086534, -1.315889)])
        robot.point_queue_callback([(-11.521123, -3.106407)])
        robot.point_queue_callback([(-11.251677, -5.235918)])
        robot.point_queue_callback([(-10.478114, -7.261125)])
        robot.point_queue_callback([(-9.200408, -8.895178)])
        robot.point_queue_callback([(-7.601103, -9.964276)])
        robot.point_queue_callback([(-5.593270, -10.459704)])
        robot.point_queue_callback([(-3.515941, -10.311967)])
        robot.point_queue_callback([(-1.812329, -9.686143)])
        robot.point_queue_callback([(-0.308641, -8.695256)])
        robot.point_queue_callback([(0.543160, -7.730462)])
        robot.point_queue_callback([(0.812607, -6.687454)])
        robot.point_queue_callback([(0.343249, -5.861741)])
        robot.point_queue_callback([(-0.847534, -4.983845)])
        robot.point_queue_callback([(-3.003112, -4.288496)])
        robot.point_queue_callback([(-5.601987, -4.105980)])
        robot.point_queue_callback([(-7.236040, -3.471470)])
        robot.point_queue_callback([(-8.174774, -2.463213)])
        robot.point_queue_callback([(-8.461595, -1.359348)])
        robot.point_queue_callback([(-8.148683, -0.281560)])
        robot.point_queue_callback([(-6.757986, 0.552859)])
        robot.point_queue_callback([(-5.254298, 0.787537)])
        robot.point_queue_callback([(-3.255182, 0.344254)])
        robot.point_queue_callback([(-1.638494, 0.005272)])
        robot.point_queue_callback([(0.012957, -0.012112)])

        

    except rospy.ROSInterruptException:
        pass
