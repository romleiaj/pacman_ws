#! /usr/bin/python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
    
L = 0.5 # wheelbase in meters
R = 0.127 # wheel radius in meters
GEAR_RATIO = 28.60839844
MAX_MOTOR_RPM = 3550
MAX_WHEEL_RPM = (MAX_MOTOR_RPM / GEAR_RATIO)
RPM_PER_GO = MAX_WHEEL_RPM / 1000.
RADS_PER_GO = (RPM_PER_GO*2*np.pi)/60.

def cmdvel2go(data):
    global pub, last
    last = rospy.Time.now()
    v = data.linear.x
    w = data.angular.z

    rads_right = (2*v - (L*w)) / (2.*R)
    rads_left = (2*v + (L*w)) / (2.*R)

    Rmot = (rads_right / RADS_PER_GO)
    Lmot = (rads_left / RADS_PER_GO)

    # Capping GO units
    if Rmot > 1000:
        Rmot = 1000
    elif Rmot < -1000:
        Rmot = -1000
    if Lmot > 1000:
        Lmot = 1000
    elif Lmot < -1000:
        Lmot = -1000
    msg = Int32MultiArray()
    msg.data = [Lmot, Rmot]
    pub.publish(msg)

def main():
    rospy.init_node("cmdvel2GO")
    global pub, last
    pub_topic = rospy.get_param("~pub_topic", default="roboteq_cmd")
    pub = rospy.Publisher(pub_topic, Int32MultiArray, queue_size=10)    
    last = rospy.Time.now()

    rospy.Subscriber("/cmd_vel", Twist, cmdvel2go)
    
    while not rospy.is_shutdown(): # heartbeat
        if (rospy.Time.now() - last).to_sec() > 0.5:
            rospy.logwarn("No input detected on /cmd_vel, stopping motors.")
            msg = Int32MultiArray()
            msg.data = [0, 0]
            pub.publish(msg)
        rospy.sleep(0.1)
            
    rospy.loginfo("Waiting for messages on /cmd_vel...")

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
        pass

