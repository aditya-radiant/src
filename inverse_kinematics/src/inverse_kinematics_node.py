#!/usr/bin/env python
import rospy
import math
import numpy as np
from dynamixel_control.msg import DynamixelPos, DynamixelPosList
from inverse_kinematics.msg import IKTarget

l1 = 13.9
l2 = 14.6

dynamixelPosList_pub = rospy.Publisher('set_position', DynamixelPosList, queue_size=10)


def r_leg_ik(koordinatX, koordinatY,  koordinatZ, rotate):
    x = koordinatX
    y = koordinatY
    z = koordinatZ

    try:
        A = (x**2 + y**2 + z**2 - l1**2 - l2**2) /(2*l1*l2)
        B = math.sqrt(1-(A**2))
        tetha3 = math.degrees(math.acos(A))
        
        buff1 = l2*math.cos(tetha3) + l1
        buff2 = math.sqrt(x**2 + y**2)
        buff3 = l2*math.sin(tetha3)

        C = math.degrees(math.atan2(buff2,z))
        D = math.degrees(math.atan2(buff3,buff1))
        tetha2 = C - D
        tetha1 = math.degrees(math.atan2(x,math.sqrt(x**2 + y**2 + z**2)))
        tetha4 = tetha3 + tetha2
        tetha6 = rotate
        tetha5 = tetha1
        return [tetha1+180, tetha2+180, -tetha3+180, tetha4+180, tetha5+180, tetha6+180]

    except ValueError:
        rospy.logerr('[IK_Target] Value Error')
        return [0, 0, 0, 0, 0, 0]
    

def l_leg_ik(koordinatX, koordinatY,  koordinatZ, rotate):
    x = koordinatX
    y = koordinatY
    z = koordinatZ

    try:
        A = (x**2 + y**2 + z**2 - l1**2 - l2**2) /(2*l1*l2)
        B = math.sqrt(1-(A**2))
        tetha3 = math.degrees(math.acos(A))
        
        buff1 = l2*math.cos(tetha3) + l1
        buff2 = math.sqrt(x**2 + y**2)
        buff3 = l2*math.sin(tetha3)

        C = math.degrees(math.atan2(buff2,z))
        D = math.degrees(math.atan2(buff3,buff1))
        tetha2 = C - D
        tetha1 = math.degrees(math.atan2(x,math.sqrt(x**2 + y**2 + z**2)))
        tetha4 = tetha3 + tetha2
        tetha6 = rotate
        tetha5 = tetha1
        return [-tetha1+180, -tetha2+180, tetha3+180, -tetha4+180, tetha5+180, -tetha6+180]

    except ValueError:
        rospy.logerr('[IK_Target] Value Error')
        return [0, 0, 0, 0, 0, 0]


def r_leg_ik_handler(data):
    result = r_leg_ik(data.point.x, data.point.y, data.point.z, data.rotate)
    id = [9, 11, 13, 15, 17, 7]

    dataPub = DynamixelPosList()

    for idx in range(6):
        dynamixelPos = DynamixelPos()
        dynamixelPos.id = id[idx]
        dynamixelPos.position = result[idx]
        dataPub.dynamixel.append(dynamixelPos)

    dynamixelPosList_pub.publish(dataPub)


def l_leg_ik_handler(data):
    result = l_leg_ik(data.point.x, data.point.y, data.point.z, data.rotate)
    id = [10, 12, 14, 16, 18, 8]

    dataPub = DynamixelPosList()

    for idx in range(6):
        dynamixelPos = DynamixelPos()
        dynamixelPos.id = id[idx]
        dynamixelPos.position = result[idx]
        dataPub.dynamixel.append(dynamixelPos)

    dynamixelPosList_pub.publish(dataPub)
    

if __name__ == '__main__' :
    rospy.init_node('inverse_kinematics_node')

    rospy.Subscriber('r_leg_target', IKTarget, r_leg_ik_handler)
    rospy.Subscriber('l_leg_target', IKTarget, l_leg_ik_handler)

    rate = rospy.Rate(10)
    rospy.spin()