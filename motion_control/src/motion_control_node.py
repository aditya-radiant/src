#!/usr/bin/env python
import rospy, math
import numpy as np
from time import sleep
from std_msgs.msg import Bool
from dynamixel_control.msg import DynamixelPosList
from inverse_kinematics.msg import FullBodyIK

rospy.init_node('motion_control_node')
fullBodyIK_pub = rospy.Publisher('full_body_target', FullBodyIK, queue_size=10)

walk_height = 5.0   # cm
step_periode = 0.5  # second
calc_rate = 20.0    # hz


def walkingSin(t):
    return ((walk_height/2) * math.sin((2 * math.pi / step_periode) * (t - step_periode/4)) + walk_height/2)


def startWalk():
    T = np.linspace(0, step_periode, calc_rate * step_periode)
    for t in T:
        z = walkingSin(t)
        dataPub = FullBodyIK()
        dataPub.right_leg.z = 28.5 - z
        dataPub.left_leg.z = 28.5
        fullBodyIK_pub.publish(dataPub)
        sleep(1/calc_rate)
    for t in T:
        z = walkingSin(t)
        dataPub = FullBodyIK()
        dataPub.right_leg.z = 28.5
        dataPub.left_leg.z = 28.5 - z
        fullBodyIK_pub.publish(dataPub)
        sleep(1/calc_rate)


if __name__ == '__main__':
    while not rospy.is_shutdown():
        startWalk()