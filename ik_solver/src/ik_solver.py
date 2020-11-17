#!/usr/bin/env python

import math
#import numpy as np
import rospy
from std_msgs.msg import String,Float64MultiArray

l1 = 13.9
l2 = 14.6

def ik(koordinatX, koordinatY,  koordinatZ, rotate):
    x = koordinatX
    y = koordinatZ
    z = koordinatY


    A = (x**2 + y**2 + z**2 - l1**2 - l2**2) /(2*l1*l2)
    B = math.sqrt(1-(A**2))
    tetha3 = math.degrees(math.acos(A))
    
    buff1 = l2*math.cos(tetha3) + l1
    buff2 = math.sqrt(x**2 + y**2)
    buff3 = l2*math.sin(tetha3)

    C = math.degrees(math.atan2(buff2,z))      #z*buff1 + buff2*buff3
    D = math.degrees(math.atan2(buff3,buff1))  #buff1*buff2 - z*buff3
    tetha2 = C - D   #math.degrees(math.atan2(C,D))
    tetha1 = math.degrees(math.atan2(x,y))
    tetha4 = tetha3 - tetha2
    tetha6 = rotate
    tetha5 = tetha6
    return [tetha1, tetha2, tetha3, tetha4, tetha5, tetha6]

if __name__ == '__main__' :
    rospy.init_node('ik_solver_node', anonymous= False)
    rate = rospy.Rate(10)
    pub = rospy.Publisher('ik_solver_chater_node', Float64MultiArray, queue_size=10)

    while not rospy.is_shutdown():
        data_to_send = Float64MultiArray()  # the data to be sent, initialise the array
        data_to_send.data = ik(5,5,5, 10) # assign the array with the value you want to send
        pub.publish(data_to_send)
        print(data_to_send)
        rate.sleep()