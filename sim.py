
from PyQt5 import QtGui
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QPainter, QBrush, QPen
from PyQt5.QtCore import Qt, QRunnable, QThreadPool, QTimer
import numpy as np
import sys
import argparse
import Pyro4
from Pyro4.errors import CommunicationError
import os, sys, struct


from utils import *
from config import *
from robot import Arm
from ball import Ball
class Window(QMainWindow):

    def __init__(self, sim_id, control_params, sim_time):

        super().__init__()
        self.title = "3R Pitcher"
        self.top= 150
        self.left= 150
        self.width = WIDTH
        self.height = HEIGHT

        self.sim_id = sim_id
        sys.excepthook = Pyro4.util.excepthook
        self.simco_server = Pyro4.Proxy("PYRO:interface@localhost:53546")
        self.arm = Arm()
        self.arm.mode = THROW
        self.ball = Ball(self.arm, M_OBJECT, [1.0, 1.0], [0, 0], [0, 0])
        self.clock = 0
        self.servo = SERVO_RATE
        self.render = RENDER_RATE
        self.control_params = control_params
        self.arm.update_state()
        self.arm.simulate()
        self.ball.simulate()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_control)
        self.timer.start(sim_time)
        self.spline_log_file = open('results/sim_{}_spline.txt'.format(self.sim_id), 'w')
        self.ball_pos_log_file = open('results/sim_{}_ball_pos.txt'.format(self.sim_id), 'w')
        self.InitWindow()

    def InitWindow(self):

        self.setWindowTitle(self.title)
        self.setGeometry(self.top, self.left, self.width, self.height)
        self.show()

    def update_control(self):
        #if self.servo == SERVO_RATE:
        self.arm.control(self.clock, control_params)
        #self.servo = 1
        #self.servo += 1
        self.arm.simulate()
        self.ball.simulate()
        if self.render == RENDER_RATE:
            self.update()
            self.render = 1
        self.render += 1
        self.clock += DT
        self.spline_log_file.write(str(self.arm.links[1].theta) + ', ' +
                                   str(self.arm.links[2].theta) + ', ' +
                                   str(self.arm.links[3].theta) + ', ' +
                                   str(self.arm.links[1].theta_dot) + ', ' +
                                   str(self.arm.links[2].theta_dot) + ', ' +
                                   str(self.arm.links[3].theta_dot) + ', ' +
                                   str(self.arm.links[1].torque) + ', ' +
                                   str(self.arm.links[2].torque) + ', ' +
                                   str(self.arm.links[3].torque) +
                                   '\n')
        self.ball_pos_log_file.write(str(self.ball.position[X]) + ', ' +
                                str(self.ball.position[Y]) + '\n')
        if(self.arm.release and np.abs(self.ball.position[Y])<0.001):
            self.simco_server.report_result({'id': self.sim_id, 'distance': self.ball.position[X]})
            print('Sim {} done with distance {}'.format(self.sim_id, self.ball.position[X]))
            exit(0)

    def paintEvent(self, event):

        painter = QPainter(self)
        self.drawAll(painter)

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
    def drawBall(self, painter):
        painter.drawEllipse(W2DX(self.ball.position[X]),
                            W2DY(self.ball.position[Y]),
                            2*W2DR(R_OBJ),
                            2*W2DR(R_OBJ))
    def drawAll(self, painter):
        self.drawRobot(painter)
        self.drawBall(painter)
if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Short sample app')

    parser.add_argument('--delta_t1', required=False, type=float)
    parser.add_argument('--delta_t2', required=False, type=float)
    parser.add_argument('--delta_t3', required=False, type=float)
    parser.add_argument('--sim_id', required=False, default=0, type=int)
    parser.add_argument('--sim_time', required=False, default=5.0, type=float)


    args = parser.parse_args()

    control_params = [0.0, args.delta_t1, args.delta_t2, args.delta_t3]
    if None in control_params:
        control_params = None

    App = QApplication(sys.argv)
    window = Window(args.sim_id, control_params, args.sim_time)
    sys.exit(App.exec())
