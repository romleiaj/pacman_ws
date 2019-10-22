#! /usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

def cmdvel2go(data):
    global pub
    v = data.linear.x
    w = (data.angular.z) * 2

    scale = 500
    Lmot = ((v + w) / 2) * scale
    Rmot = ((v - w) / 2) * scale

    if Rmot > 500:
        Rmot = 500
    if Lmot > 500:
        Lmot = 500
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

