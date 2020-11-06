#!/usr/bin/env python

import rospy
from std_msgs.msg import String


if __name__ == '__main__' :
    rospy.init_node('ik_solver_node', anonymous= False)
    rate = rospy.Rate(10)
    pub = rospy.Publisher('ik_solver_chatter_node', String, queue_size = 1)

    while not rospy.is_shutdown():
        String_msg = 'kiriman dari talker python node'
        pub.publish(String_msg)
        rate.sleep()