#!/usr/bin/env python

import rospy

from std_msgs.msg import String,Float64MultiArray

def ik_solver_callback(msg):
    rospy.loginfo('Listener menerima data : ', msg.data)
 


if __name__ == '__main__' :
    rospy.init_node('ik_listener_node', anonymous=False)

    rospy.Subscriber('ik_solver_chatter_node', Float64MultiArray, ik_solver_callback)
    rospy.spin() #biar ga hanya berjalan sekali