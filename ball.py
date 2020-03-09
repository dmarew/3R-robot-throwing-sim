
from robot import Arm
import numpy as np
from config import *
class Ball:
    def __init__(self, arm, mass, position, velocity, ext_force):
        self.arm = arm
        self.mass = mass
        self.position = position
        self.velocity = velocity
        self.ext_force = ext_force

    def simulate(self):
        if not self.arm.release:
            c1 = np.cos(self.arm.links[1].theta)
            s1 = np.sin(self.arm.links[1].theta)
            c12 = np.cos(self.arm.links[1].theta + self.arm.links[2].theta)
            s12 = np.sin(self.arm.links[1].theta + self.arm.links[2].theta)
            c123 = np.cos(self.arm.links[1].theta + self.arm.links[2].theta + self.arm.links[3].theta)
            s123 = np.sin(self.arm.links[1].theta + self.arm.links[2].theta + self.arm.links[3].theta)
            self.position[X] = L1*c1+L2*c12+L3*c123
            self.position[Y] = L1*s1+L2*s12+L3*s123
            self.velocity[X] = (-L1*s1-L2*s12-L3*s123)*self.arm.links[1].theta_dot + (-L2*s12-L3*s123)*self.arm.links[2].theta_dot + (-L3*s123)*self.arm.links[3].theta_dot
            self.velocity[Y] = (L1*c1+L2*c12+L3*c123)*self.arm.links[1].theta_dot + (L2*c12+L3*c123)*self.arm.links[2].theta_dot + (L3*c123)*self.arm.links[3].theta_dot
        else:
            self.velocity[Y] += (-GRAVITY) * DT
            self.position[X] += self.velocity[X]*DT
            self.position[Y] += 0.5*(-GRAVITY)*SQR(DT) + self.velocity[Y]*DT
            if self.position[Y] < (GROUND_PLANE_Y+R_OBJ) and self.velocity[Y] < 0.0:
                self.velocity[Y] *= -1.0

if __name__ == '__main__':
    ball = Ball(Arm(), M_OBJECT, [1, 1], [0, 0])
