#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist, Vector3
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt,pi
from turtlesim.msg import Pose
from PID import PID

import numpy as np
import tf
import pdb
import Queue
import matplotlib.pyplot as plt


class PathNavigation():

    def __init__(self):
        #Creating node,publisher and subscriber
        rospy.init_node('PathNav_node', anonymous=True)
        self.velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.odom_subscriber = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        self.odom = Odometry()
        #self.point_subscriber = rospy.Subscriber(filtered_points_topic, tuple, self.point_queue_callback)
        self.queue = []
        self.rate = rospy.Rate(100)
        #self.ang_vel = PID()

    #Callback function implementing the odom value received
    def odom_callback(self, data):
        self.odom = data
        
    def point_queue_callback(self, array): #FIFO Queue
        #dimension = np.asarray(array).shape
        for point in array:
            self.queue.insert(0, point)
        print("Q: " + str(len(self.queue)))
        
    def get_next_point(self):
        if len(self.queue) < 1: # No points in queue
            print("no points in queue")
            raise Queue.Empty
        else:
            return self.queue.pop()
        print("Q: " + str(len(self.queue)))
        
    def get_distance(self, goal_x, goal_y):
        pose = self.odom.pose.pose.position
        distance = sqrt(pow((goal_x - pose.x), 2) + pow((goal_y - pose.y), 2))
        #print("Distance: " + str(distance))
        return distance

    def move2point(self):
        pose = self.odom.pose.pose.position
        goal_point = self.get_next_point()
        #goal_point = np.asarray(goal_point) #I dont think this is needed
        goal_pose_x = goal_point[0] # x value
        goal_pose_y = goal_point[1] # y value
        vel_msg = Twist()

        # User specified tolerance and gains
        distance_tolerance = 0.6
        lin_vel = .2 #.2 worked
        Kp_w = 6
        #Ki_w = 1
        #self.ang_vel.setPoint(0)
        #self.ang_vel.Kp = 0
        #self.ang_vel.Kd = 0
        #self.ang_vel.Ki = 0
        
        while self.get_distance(goal_pose_x, goal_pose_y) >= distance_tolerance:
            orient = self.odom.pose.pose.orientation
            [roll, pitch, yaw] = tf.transformations.euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
            theta = atan2(goal_pose_y - pose.y, goal_pose_x - pose.x)
            
            #Porportional Controller
            
            if len(self.queue) >= 0:
                #linear velocity in the x-axis:
                turn_angle = pi/2
                turn_factor = (turn_angle - abs(theta - yaw)) / turn_angle
                if turn_factor < 0:
                    turn_factor = 0
                vel_msg.linear.x = lin_vel * turn_factor
                #print(vel_msg.linear.x)
                print(turn_factor)
                #angular velocity in the z-axis: Add Differential
                vel_msg.angular.z = Kp_w * (theta - yaw)
                
            else: # Last element of queue
                vel_msg.linear.x = 0
                vel_msg.angular.z =0
                #vel_msg.linear.x = lin_vel * sqrt(pow((goal_pose_x - pose.x), 2) + pow((goal_pose_y - pose.y), 2))
                
            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        self.velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        #Testing our function
        cmd_vel_topic = rospy.get_param('~cmd_vel_topic', 'pacman_equiv/cmd_vel')
        odom_topic = rospy.get_param('~odom_topic', 'pacman_equiv/odom')
        robot = PathNavigation()
        #print(cmd_vel_topic)
        
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
        #arr = np.asarray(robot.queue)
        #x = [pt[0] for pt in arr]
        #y = [pt[1] for pt in arr]
        #plt.plot(x, y, 'ro')
        #plt.show()
        
        # while  true:
        #print(robot.queue)
        while len(robot.queue) > 0:
            robot.move2point()
        
        rospy.spin()

    except rospy.ROSInterruptException: pass
