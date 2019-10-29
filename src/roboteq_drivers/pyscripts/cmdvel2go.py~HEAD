#! /usr/bin/python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
    
L = 0.5 # wheelbase in meters
R = 0.127 # wheel radius in meters
GEAR_RATIO = 26.25
MAX_RPM = (7100 / GEAR_RATIO)
RPM_PER_GO = MAX_RPM / 1000.
RADS_PER_GO = (RPM_PER_GO*2*np.pi)/60.

def cmdvel2go(data):
    global pub
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
    global pub
    pub = rospy.Publisher("/roboteq_motor_command", Int32MultiArray, queue_size=10)
    
    rospy.Subscriber("/cmd_vel", Twist, cmdvel2go)
    
    rospy.loginfo("Waiting for messages on /cmd_vel...")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
        pass

