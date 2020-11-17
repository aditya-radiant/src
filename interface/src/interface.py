#!/usr/bin/env python2
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtWidgets import QApplication
import rospy, sys
from std_msgs.msg import UInt8, UInt16
from motion_control.msg import walkParam
from inverse_kinematics.msg import offset


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        uic.loadUi("/home/didik/Bascorro_ws/src/interface/src/main.ui", self)

        self.stop_btn.clicked.connect(lambda: self.motionBtnClick(0))
        self.steadyWalk_btn.clicked.connect(lambda: self.motionBtnClick(1))
        self.walk_btn.clicked.connect(lambda: self.motionBtnClick(2))
        self.turnRight_btn.clicked.connect(lambda: self.motionBtnClick(3))
        self.turnLeft_btn.clicked.connect(lambda: self.motionBtnClick(4))
        self.apply_btn.clicked.connect(self.applyBtnClick)
        self.camPitch_slider.valueChanged.connect(self.camPitchChanged)

    def motionBtnClick(self, data):
        motion_pub.publish(data)

    def applyBtnClick(self):
        dataPub = walkParam()
        dataPub.step_distance = float(self.stepDist_line.text())
        dataPub.step_height = float(self.stepHeight_line.text())
        dataPub.step_periode = float(self.stepPeriode_line.text())
        dataPub.swap_distance = float(self.swapDist_line.text())
        dataPub.turn_angle = float(self.turnAng_line.text())
        walkParam_pub.publish(dataPub)

        dataPub = offset()
        dataPub.front_offset = float(self.frontOffset_line.text())
        dataPub.side_offset = float(self.sideOffset_line.text())
        offset_pub.publish(dataPub)

    def camPitchChanged(self):
        data = self.camPitch_slider.value()
        camPitch_pub.publish(data)


rospy.init_node('interface')
motion_pub = rospy.Publisher('set_motion', UInt8, queue_size=1)
camPitch_pub = rospy.Publisher('set_camPitch', UInt16, queue_size=1)
walkParam_pub = rospy.Publisher('set_walkParam', walkParam, queue_size=1)
offset_pub = rospy.Publisher('set_offset', offset, queue_size=1)

app = QApplication(sys.argv) 
form = MainWindow()

form.show()
app.exec_()
