
from PyQt5 import QtGui
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QPainter, QBrush, QPen
from PyQt5.QtCore import Qt, QRunnable, QThreadPool, QTimer
import numpy as np
import sys
from utils import *
from config import *
from robot import Arm
class Window(QMainWindow):

    def __init__(self):

        super().__init__()
        self.title = "3R Pitcher"
        self.top= 150
        self.left= 150
        self.width = WIDTH
        self.height = HEIGHT
        self.arm = Arm()
        self.arm.mode = THROW
        self.clock = 0
        self.servo = SERVO_RATE
        self.render = RENDER_RATE
        self.arm.update_state()
        self.arm.simulate()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_control)
        self.timer.start(10000*DT)

        self.InitWindow()

    def InitWindow(self):

        self.setWindowTitle(self.title)
        self.setGeometry(self.top, self.left, self.width, self.height)
        self.show()

    def update_control(self):
        #if self.servo == SERVO_RATE:
        #self.arm.control()
        #self.servo = 1
        #self.servo += 1
        self.arm.simulate()

        if self.render == RENDER_RATE:
            self.update()
            self.render = 1
        self.render += 1
        self.clock += DT
        #print('timer ', self.clock)
        print('torques: ', self.arm.links[1].torque, self.arm.links[2].torque, self.arm.links[3].torque)

    def paintEvent(self, event):

        painter = QPainter(self)
        self.drawRobot(painter)

    def drawRobotChassis(self, painter):
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(Qt.blue)
        painter.setBrush(Qt.white)
        painter.drawLine(W2DX(-0.0758), W2DY(GROUND_PLANE_Y+0.1010),
                         W2DX(0.0758), W2DY(GROUND_PLANE_Y+0.1010))
        painter.drawLine(W2DX(0.0758), W2DY(GROUND_PLANE_Y+0.1010),
                         W2DX(0.0758), W2DY(GROUND_PLANE_Y+0.5556))
        painter.drawLine(W2DX(0.0758), W2DY(GROUND_PLANE_Y+0.5556),
                         W2DX(-0.0758), W2DY(GROUND_PLANE_Y+0.5556))
        painter.drawLine(W2DX(-0.0758), W2DY(GROUND_PLANE_Y+0.5556),
                         W2DX(-0.0758), W2DY(GROUND_PLANE_Y+0.1010))
        painter.setBrush(QBrush(Qt.blue, Qt.SolidPattern))
        painter.drawEllipse(W2DX(0.0)-W2DR(0.1010), W2DY(GROUND_PLANE_Y+0.1010), W2DR(2*0.1010), W2DR(2*0.1010))
    def drawRobot(self, painter):
        self.drawRobotChassis(painter)
        temp0 = self.arm.links[0].iTj
        for i in range(1, NFRAMES):
            temp1 = np.matmul(temp0, self.arm.links[i].iTj)
            painter.setPen(Qt.red)
            painter.setBrush(QBrush(Qt.red, Qt.SolidPattern))
            painter.drawLine(W2DX(temp0[0][3]), W2DY(temp0[1][3]),
                    	     W2DX(temp1[0][3]), W2DY(temp1[1][3]))
            painter.drawEllipse(W2DX(temp1[0][3])-W2DR(R_JOINT),
                                W2DY(temp1[1][3])-W2DR(R_JOINT),
                                2*W2DR(R_JOINT),
                                2*W2DR(R_JOINT))
            temp0 = temp1
    def drawBall(self):
        pass
    def drawAll(self):
        painter = QPainter(self)
        self.drawRobot(painter)
        self.drawBall()
if __name__ == '__main__':
    App = QApplication(sys.argv)
    window = Window()
    sys.exit(App.exec())
