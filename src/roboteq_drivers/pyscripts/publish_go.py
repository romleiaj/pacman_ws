#! /usr/bin/python
import rospy
import time
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension

def talker():
    print("MADE IT")
    rospy.init_node("pub_mot")
    pub = rospy.Publisher("/roboteq_motor_command", Int32MultiArray, queue_size=10)
    msg = Int32MultiArray()
    #dim = MultiArrayDimension()
    #lay = MultiArrayLayout()
    #dim.label = ''
    #dim.stride = 0
    msg.data = [0, 0]
    #lay.dim.append(dim)
    #lay.data_offset = 0
    #msg.layout = lay
    print("PUBLISHING")
    print(msg)
    for i in range(5):
        pub.publish(msg)
        time.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
