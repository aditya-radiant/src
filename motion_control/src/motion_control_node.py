#!/usr/bin/env python
import rospy, math
import numpy as np
from time import sleep
from std_msgs.msg import Bool, UInt8
from dynamixel_control.msg import DynamixelPosList
from inverse_kinematics.msg import FullBodyIK

rospy.init_node('motion_control_node')
fullBodyIK_pub = rospy.Publisher('full_body_target', FullBodyIK, queue_size=10)

walk_height = 5.0   # cm
step_periode = 0.25  # second
calc_rate = 60.0    # hz
step_distance = 8.0 # cm
side_distance = 3.0 # cm
rotate_distance = 20 # degree

motion_state = 0    #start_pose


def walkingSinZ(t):
    return ((walk_height/2) * math.sin((2 * math.pi / step_periode) * (t - step_periode/4)) + walk_height/2)


def walkingSinX(t):
    return ((side_distance/2) * math.sin((2 * math.pi / step_periode) * (t/2 - step_periode/4)) + side_distance/2)


def startWalk():
    T = np.linspace(0, step_periode, calc_rate * step_periode)
    for t in T: 	#left foot side
        x = walkingSinX(t)
        dataPub = FullBodyIK()
        dataPub.right_leg.x = x
        dataPub.right_leg.z = 26.0
        dataPub.left_leg.x = -x
        dataPub.left_leg.z = 26.0
        fullBodyIK_pub.publish(dataPub)
        sleep(1/calc_rate)

    for t in T:         # right foot up
        z = walkingSinZ(t)
        y = t/step_periode * step_distance
        dataPub = FullBodyIK()
        dataPub.right_leg.x = side_distance
        dataPub.right_leg.y = y
        dataPub.right_leg.z = 26.0 - z
        dataPub.left_leg.x = -side_distance
        dataPub.left_leg.z = 26.0
        fullBodyIK_pub.publish(dataPub)
        sleep(1/calc_rate)

    for t in T: 	#left foot side
        x = walkingSinX(t)
        dataPub = FullBodyIK()
        dataPub.right_leg.x = -x
        dataPub.right_leg.z = 26.0
        dataPub.left_leg.x = x
        dataPub.left_leg.z = 26.0
        fullBodyIK_pub.publish(dataPub)
        sleep(1/calc_rate)

    for t in T:         # left foot up
        z = walkingSinZ(t)
        y = t/step_periode * step_distance
        dataPub = FullBodyIK()
        dataPub.right_leg.x = -side_distance
        dataPub.right_leg.z = 26.0
        dataPub.left_leg.x = side_distance
        dataPub.left_leg.y = y
        dataPub.left_leg.z = 26.0 - z
        fullBodyIK_pub.publish(dataPub)
        sleep(1/calc_rate)

def turnRight():
    T = np.linspace(0, step_periode, calc_rate * step_periode)
    for t in T: 	#left foot side
        x = walkingSinX(t)
        dataPub = FullBodyIK()
        dataPub.right_leg.x = x
        dataPub.right_leg.z = 26.0
        dataPub.left_leg.x = -x
        dataPub.left_leg.z = 26.0
        fullBodyIK_pub.publish(dataPub)
        sleep(1/calc_rate)

    for t in T:         # right foot up
        z = walkingSinZ(t)
        y = t/step_periode * step_distance
	rotate = t/step_periode * rotate_distance
        dataPub = FullBodyIK()
        dataPub.right_leg.x = side_distance
        dataPub.right_leg.y = y
        dataPub.right_leg.z = 26.0 - z
	dataPub.right_leg.rotate = rotate
        dataPub.left_leg.x = -side_distance
        dataPub.left_leg.z = 26.0
        fullBodyIK_pub.publish(dataPub)
        sleep(1/calc_rate)

    for t in T: 	#left foot side
        x = walkingSinX(t)
        dataPub = FullBodyIK()
        dataPub.right_leg.x = -x
        dataPub.right_leg.z = 26.0
        dataPub.left_leg.x = x
        dataPub.left_leg.z = 26.0
        fullBodyIK_pub.publish(dataPub)
        sleep(1/calc_rate)

    for t in T:         # left foot up
        z = walkingSinZ(t)
        y = t/step_periode * step_distance
        dataPub = FullBodyIK()
        dataPub.right_leg.x = -side_distance
        dataPub.right_leg.z = 26.0
        dataPub.left_leg.x = side_distance
        dataPub.left_leg.y = y
        dataPub.left_leg.z = 26.0 - z
        fullBodyIK_pub.publish(dataPub)
        sleep(1/calc_rate)

def turnLeft():
    T = np.linspace(0, step_periode, calc_rate * step_periode)
    for t in T: 	#left foot side
        x = walkingSinX(t)
        dataPub = FullBodyIK()
        dataPub.right_leg.x = x
        dataPub.right_leg.z = 26.0
        dataPub.left_leg.x = -x
        dataPub.left_leg.z = 26.0
        fullBodyIK_pub.publish(dataPub)
        sleep(1/calc_rate)

    for t in T:         # right foot up
        z = walkingSinZ(t)
        y = t/step_periode * step_distance
        dataPub = FullBodyIK()
        dataPub.right_leg.x = side_distance
        dataPub.right_leg.y = y
        dataPub.right_leg.z = 26.0 - z
        dataPub.left_leg.x = -side_distance
        dataPub.left_leg.z = 26.0
        fullBodyIK_pub.publish(dataPub)
        sleep(1/calc_rate)

    for t in T: 	#left foot side
        x = walkingSinX(t)
        dataPub = FullBodyIK()
        dataPub.right_leg.x = -x
        dataPub.right_leg.z = 26.0
        dataPub.left_leg.x = x
        dataPub.left_leg.z = 26.0
        fullBodyIK_pub.publish(dataPub)
        sleep(1/calc_rate)

    for t in T:         # left foot up
        z = walkingSinZ(t)
        y = t/step_periode * step_distance
	rotate = t/step_periode * rotate_distance
        dataPub = FullBodyIK()
        dataPub.right_leg.x = -side_distance
        dataPub.right_leg.z = 26.0
        dataPub.left_leg.x = side_distance
        dataPub.left_leg.y = y
	dataPub.left_leg.rotate = rotate
        dataPub.left_leg.z = 26.0 - z
        fullBodyIK_pub.publish(dataPub)
        sleep(1/calc_rate)


def walkSteady():
    T = np.linspace(0, step_periode, calc_rate * step_periode)
    for t in T: 	#left foot side
        x = walkingSinX(t)
        dataPub = FullBodyIK()
        dataPub.right_leg.x = x
        dataPub.right_leg.z = 26.0
        dataPub.left_leg.x = -x
        dataPub.left_leg.z = 26.0
        fullBodyIK_pub.publish(dataPub)
        sleep(1/calc_rate)

	
    for t in T:         # right foot up
        z = walkingSinZ(t)
        dataPub = FullBodyIK()
        dataPub.right_leg.x = side_distance
        dataPub.right_leg.z = 26.0 - z
        dataPub.left_leg.x = -side_distance
        dataPub.left_leg.z = 26.0
        fullBodyIK_pub.publish(dataPub)
        sleep(1/calc_rate)

    for t in T: 	#left foot side
        x = walkingSinX(t)
        dataPub = FullBodyIK()
        dataPub.right_leg.x = -x
        dataPub.right_leg.z = 26.0
        dataPub.left_leg.x = x
        dataPub.left_leg.z = 26.0
        fullBodyIK_pub.publish(dataPub)
        sleep(1/calc_rate)

    for t in T:         # left foot up
        z = walkingSinZ(t)
        dataPub = FullBodyIK()
        dataPub.right_leg.x = -side_distance
        dataPub.right_leg.z = 26.0
        dataPub.left_leg.x = side_distance
        dataPub.left_leg.z = 26.0 - z
        fullBodyIK_pub.publish(dataPub)
        sleep(1/calc_rate)


def startPose():
    dataPub = FullBodyIK()
    dataPub.right_leg.z = 26.0
    dataPub.left_leg.z = 26.0
    fullBodyIK_pub.publish(dataPub)
    sleep(1/calc_rate)
    

def setMotionHandler(data):
    global motion_state
    motion_state = data.data
    rospy.loginfo('motion: ' + str(data.data))


if __name__ == '__main__':
    rospy.Subscriber('set_motion', UInt8, setMotionHandler)

    while not rospy.is_shutdown():
        if motion_state == 0:
            startPose()
        elif motion_state == 1:
	    walkSteady()
        elif motion_state == 2:
            startWalk()
	elif motion_state == 3:
            turnRight()
	elif motion_state == 4:
            turnLeft()
