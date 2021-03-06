import numpy as np
from config import *

class Link:
    def __init__(self, iTj, dof_type, axis, theta, theta_dot, theta_ref, theta_dot_ref, torque, ext_force):

        self.iTj = iTj
        self.dof_type= dof_type
        self.axis = axis
        self.theta = theta
        self.theta_dot  = theta_dot
        self.theta_ref = theta_ref
        self.theta_dot_ref = theta_dot_ref
        self.torque = torque
        self.ext_force = ext_force

class Arm:
    def __init__(self):

        self.release = False
        self.throw_sequence = 0
        self.joint_state = [False, False, False, False]
        self.joint_execution_time = [0.0, 0.0, 0.0, 0.0]
        self.delta_ts = [0.0, 0.2190, 0.0865, 0.02050]
        self.mode = FREEFALL
        self.setup_links()
        self.file = open('log.txt', 'w')

    def setup_links(self):

        link1_T = np.eye(4).astype('float32')
        link1 = Link(link1_T, NOTYPE, NOAXIS, 0.0, 0.0, 0.0, 0.0, 0.0, np.array([0.0, 0.0]))

        link2_T = np.eye(4).astype('float32')
        link2 = Link(link2_T, REVOLUTE, ZAXIS, THETA1_HOME, 0.0, THETA1_REF, 0.0, 0.0, np.array([0.0, 0.0]))

        link3_T = np.eye(4).astype('float32')
        link3_T[0, -1] = L1
        link3 = Link(link3_T, REVOLUTE, ZAXIS, THETA2_HOME, 0.0, THETA2_REF, 0.0, 0.0, np.array([0.0, 0.0]))

        link4_T = np.eye(4).astype('float32')
        link4_T[0, -1] = L2
        link4 = Link(link4_T, REVOLUTE, ZAXIS, THETA3_HOME, 0.0, THETA3_REF, 0.0, 0.0, np.array([0.0, 0.0]))

        link5_T = np.eye(4).astype('float32')
        link5_T[0, -1] = L3
        link5 = Link(link5_T, NOTYPE, NOAXIS, 0.0, 0.0, 0.0, 0.0, 0.0, np.array([0.0, 0.0]))
        self.links = [link1, link2, link3, link4, link5]

    def dynamics(self):
        l = [L1, L2, L3]
        m = [M1, M2, M3]

        s1 = np.sin(self.links[1].theta)
        c1 = np.cos(self.links[1].theta)
        s2 = np.sin(self.links[2].theta)
        c2 = np.cos(self.links[2].theta)
        s3 = np.sin(self.links[3].theta)
        c3 = np.cos(self.links[3].theta)
        s12 = np.sin(self.links[1].theta + self.links[2].theta)
        c12 = np.cos(self.links[1].theta + self.links[2].theta)
        s23 = np.sin(self.links[2].theta + self.links[3].theta)
        c23 = np.cos(self.links[2].theta + self.links[3].theta)
        s123 = np.sin(self.links[1].theta + self.links[2].theta + self.links[3].theta)
        c123 = np.cos(self.links[1].theta + self.links[2].theta + self.links[3].theta)

        if (self.release==False):
            m3= M3 + M_OBJECT
        else:
            m3=M3
        M = np.zeros((NJOINTS, NJOINTS))
        V = np.zeros(NJOINTS)
        G = np.zeros(NJOINTS)
        F = np.zeros(NJOINTS)
        M[0][0] = 2*M1*SQR(L1) + M2*(SQR(L1)+ 2.0*L1*L2*c2+2*SQR(L2)) + m3*(SQR(L1)+2.0*L1*L2*c2+2.0*L1*L3*c23*SQR(L2)+2.0*L2*L3*c3+SQR(L3))
        M[0][1] = M2*(L1*L2*c2 + 2*SQR(L2)) + m3*(L1*L2*c2+L1*L3*c23+SQR(L2)+2.0*L2*L3*c3+2*SQR(L3))
        M[0][2] = m3*(L1*L3*c23 + L2*L3*c3 + 2*SQR(L3))

        M[1][0] = M[0][1]
        M[1][1] = 2*M2*SQR(L2) + m3*(SQR(L2) + 2.0*L2*L3*c3 + 2*SQR(L3))
        M[1][2] = m3*(L2*L3*c3 + 2*SQR(L3))

        M[2][0] = M[0][2]
        M[2][1] = M[1][2]
        M[2][2] = 2*m3*SQR(L3)

        V[0] = -((M2+m3)*L1*L2*s2 + m3*L1*L3*s23)*SQR(self.links[2].theta_dot)
        - m3*(L1*L3*s23 + L2*L3*s3)*SQR(self.links[3].theta_dot)
        - 2.0*((M2+m3)*L1*L2*s2+m3*L1*L3*s23)*self.links[1].theta_dot*self.links[2].theta_dot
        - 2.0*m3*(L1*L3*s23 + L2*L3*s3)*self.links[1].theta_dot*self.links[3].theta_dot
        - 2.0*m3*(L1*L3*s23 + L2*L3*s3)*self.links[2].theta_dot*self.links[3].theta_dot

        V[1] = ((M2+m3)*L1*L2*s2 + m3*L1*L3*s23)*SQR(self.links[1].theta_dot)
        -(m3*L2*L3*s3)*SQR(self.links[3].theta_dot)
        -(2.0*m3*L2*L3*s3)*self.links[1].theta_dot*self.links[3].theta_dot
        -(2.0*m3*L2*L3*s3)*self.links[2].theta_dot*self.links[3].theta_dot

        V[2] = m3*(L1*L3*s23 + L2*L3*s3)*SQR(self.links[1].theta_dot)
        + m3*(L2*L3*s3)*SQR(self.links[2].theta_dot)
        + 2.0*m3*L2*L3*s3*self.links[1].theta_dot*self.links[2].theta_dot

        G[0] = (M1*L1*c1+M2*(L1*c1+L2*c12)+m3*(L1*c1+L2*c12+L3*c123))*GRAVITY
        G[1] = ((M2+m3)*L2*c12 + m3*L3*c123)*GRAVITY
        G[2] = m3*L3*c123*GRAVITY

        F[0] = F[1] = F[2] = 0.0
        return M, V, G, F
    def accelerations(self, M, V, G, F):
      args = []
      Minv = np.linalg.inv(M)
      for i in range(NJOINTS):
          args.append(self.links[i + 1].torque - V[i] - G[i] - F[i])
      theta_ddot = np.matmul(Minv, np.array(args))

      self.file.write('theta ddot: ' + np.array_str(theta_ddot)+
                      ' V: ' + np.array_str(V) +
                      ' G: ' + np.array_str(G) +
                      ' M: ' + np.array_str(M) +
                      '\n')
      return theta_ddot
    def euler(self, acc):
        # update positions and velocities of roger's manipulator
        joint = 0
        l = [L1, L2, L3]
        for i in range(NFRAMES):
            if (self.links[i].dof_type != NOTYPE):
                if self.links[i].dof_type == REVOLUTE:
                    self.links[i].theta += 0.5*acc[joint]*DT*DT + self.links[i].theta_dot*DT
                    self.links[i].theta_dot += acc[joint]*DT
                    joint += 1
                elif self.links[i].dof_type == PRISMATIC:
                    self.links[i].theta += 0.5*acc[joint]*DT*DT + self.links[i].theta_dot*DT
                    self.links[i].theta_dot += acc[joint]*DT
                    if self.links[i].theta < 0.0:
                        self.links[i].theta = 0.0
                        self.links[i].theta_dot = 0.0
                    if self.links[i].theta > l[joint+1]:
                        self.links[i].theta = l[joint+1]
                        self.links[i].theta_dot = 0.0
                    joint += 1
            self.file.write('link '+ str(i) + ' theta: ' +str(self.links[i].theta)+
                      ' theta dot: ' + str(self.links[i].theta_dot) +
                      ' theta ref: ' +str(self.links[i].theta_ref) +
                      ' theta dot ref: ' + str(self.links[i].theta_dot_ref) +
                      '\n')
        self.rectify_theta()
        self.update_state()
    def rectify_theta(self):
          for i in range(NJOINTS):
              while self.links[i].theta < -M_PI:
                  self.links[i].theta += 2.0*M_PI
              while self.links[i].theta > M_PI:
                  self.links[i].theta -= 2.0*M_PI
    def update_state(self):
      # update transforms that describe configuration of the robot
        for i in range(NFRAMES):
            if self.links[i].axis == XAXIS:
                if (self.links[i].dof_type == REVOLUTE):
                    self.links[i].iTj[1][1] =  np.cos(self.links[i].theta)
                    self.links[i].iTj[1][2] = -np.sin(self.links[i].theta)
                    self.links[i].iTj[2][1] = -self.links[i].iTj[1][2]
                    self.links[i].iTj[2][2] =  self.links[i].iTj[1][1]
                elif (self.links[i].dof_type == PRISMATIC):
                    self.links[i].iTj[0][3] = self.links[i].theta

            elif self.links[i].axis == YAXIS:
                if (self.links[i].dof_type == REVOLUTE):
                    self.links[i].iTj[0][0] = np.cos(self.links[i].theta)
                    self.links[i].iTj[0][2] = np.sin(self.links[i].theta)
                    self.links[i].iTj[2][0] = -self.links[i].iTj[0][2]
                    self.links[i].iTj[2][2] =  self.links[i].iTj[0][0]
                elif (self.links[i].dof_type == PRISMATIC):
                    self.links[i].iTj[1][3] = self.links[i].theta

            elif self.links[i].axis==ZAXIS:
                if self.links[i].dof_type == REVOLUTE:
                    self.links[i].iTj[0][0] = np.cos(self.links[i].theta)
                    self.links[i].iTj[0][1] = -np.sin(self.links[i].theta)
                    self.links[i].iTj[1][0] = -self.links[i].iTj[0][1]
                    self.links[i].iTj[1][1] =  self.links[i].iTj[0][0]

                elif self.links[i].dof_type == PRISMATIC:
                    self.links[i].iTj[2][3] = l[i-1] + self.links[i].theta
    def simulate(self):
        self.euler(self.accelerations(*self.dynamics()))

if __name__ == '__main__':
    arm = Arm()
    arm.simulate()
