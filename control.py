from config import *
import numpy as np
def control(arm, clock, control_params):
    if control_params is not None:
        arm.delta_ts = control_params
    joint = 0
    if arm.mode==FREEFALL:
        for i in range(NJOINTS):
            arm.links[i].torque=0.0
        return
    elif arm.mode==PD_CONTROL:

        for i in range(NARMS):
            acc = np.zeros(3)
            for j in range(NFRAMES):
                if (arm.links[j].dof_type == REVOLUTE):
                    #PDcontrol - desired accelerations
                    theta_error = arm.links[j].theta_ref - arm.links[j].theta
                    if theta_error > M_PI: theta_error -= 2.0*M_PI
                    if theta_error < -M_PI: theta_error += 2.0*M_PI
                    acc[joint] = KP_ARM*(theta_error) + KD_ARM*(arm.links[j].theta_dot_ref - arm.links[j].theta_dot)
                    joint += 1
            M,V,G,F = arm.dynamics()
            tmp = np.matmul(M, acc)

            joint=0
            for j in range(NFRAMES):
                if arm.links[j].dof_type == REVOLUTE:
                    arm.links[j].torque = tmp[joint] +V[joint] + G[joint]
                    joint += 1
    elif arm.mode==THROW:
        #print(arm.throw_sequence)
        #if arm.throw_sequence == 0:
        if arm.joint_state[1] and (clock-arm.joint_execution_time[1])>arm.delta_ts[2]:
            arm.throw_sequence = 2
        elif arm.joint_state[0] and (clock-arm.joint_execution_time[0])>arm.delta_ts[1]:
            arm.throw_sequence = 1
        else:
            arm.throw_sequence = 0
        # print('throw_sequence: {} joint state {} clock {} '.format(arm.throw_sequence, arm. joint_state, clock))
        if arm.throw_sequence == 0:

            arm.links[1].torque = -TORQUE_MAX
            if not arm.joint_state[0]:
                # print('first joint started excution at ', clock)
                arm.joint_state[0] = True
                arm.joint_execution_time[0] = clock
            # PDcontrol - desired accelerations
            theta_error = arm.links[2].theta_ref - arm.links[2].theta
            if theta_error > M_PI: theta_error -= 2.0*M_PI
            if theta_error < -M_PI: theta_error += 2.0*M_PI
            torq = KP_ARM*(theta_error) + KD_ARM*(arm.links[2].theta_dot_ref - arm.links[2].theta_dot)
            if torq>TORQUE_MAX: torq=TORQUE_MAX
            if torq<-TORQUE_MAX: torq=-TORQUE_MAX
            arm.links[2].torque = torq

            theta_error = arm.links[3].theta_ref - arm.links[3].theta
            if theta_error > M_PI: theta_error -= 2.0*M_PI
            if theta_error < -M_PI: theta_error += 2.0*M_PI
            torq = KP_ARM*(theta_error) + KD_ARM*(arm.links[3].theta_dot_ref - arm.links[3].theta_dot)
            if torq>TORQUE_MAX: torq=TORQUE_MAX
            if torq<-TORQUE_MAX: torq=-TORQUE_MAX
            arm.links[3].torque = torq

            # if ((arm.links[1].theta < M_PI) and (arm.links[1].theta > M_PI/2)):
                # arm.throw_sequence = 1
        elif arm.throw_sequence==1:
            if not arm.joint_state[1]:
                # print('delta_t1', clock-arm.joint_execution_time[0])
                arm.joint_state[1] = True
                arm.joint_execution_time[1] = clock
            arm.links[1].torque = -TORQUE_MAX
            arm.links[2].torque = -TORQUE_MAX

            theta_error = arm.links[3].theta_ref - arm.links[3].theta
            if theta_error > M_PI: theta_error -= 2.0*M_PI
            if theta_error < -M_PI: theta_error += 2.0*M_PI
            torq = KP_ARM*(theta_error) + KD_ARM*(arm.links[3].theta_dot_ref - arm.links[3].theta_dot)
            if torq>TORQUE_MAX: torq = TORQUE_MAX
            if torq<-TORQUE_MAX: torq = -TORQUE_MAX
            arm.links[3].torque = torq

            # if (((arm.links[1].theta+arm.links[2].theta) < 3.0*M_PI/4.0) and
            # (arm.links[1].theta > 0.0)):arm.throw_sequence = 2
        elif arm.throw_sequence==2:
            if not arm.joint_state[2]:
                # print('delta_t2', clock-arm.joint_execution_time[1])
                arm.joint_state[2] = True
                arm.joint_execution_time[2] = clock
            arm.links[1].torque = -TORQUE_MAX
            arm.links[2].torque = -TORQUE_MAX
            arm.links[3].torque = -TORQUE_MAX

            dxdt3 = -L3*np.sin(arm.links[1].theta+arm.links[2].theta+arm.links[3].theta)

            dxdt2 = -L2*np.sin(arm.links[1].theta+arm.links[2].theta) + dxdt3

            dxdt1 = -L1*np.sin(arm.links[1].theta) +dxdt2

            dydt3 = L3*np.cos(arm.links[1].theta+arm.links[2].theta+arm.links[3].theta)
            dydt2 = L2*np.cos(arm.links[1].theta+arm.links[2].theta) + dydt3
            dydt1 = L1*np.cos(arm.links[1].theta) + dydt2

            xdot = dxdt1*arm.links[1].theta_dot + dxdt2*arm.links[2].theta_dot + dxdt3*arm.links[3].theta_dot
            ydot = dydt1*arm.links[1].theta_dot + dydt2*arm.links[2].theta_dot + dydt3*arm.links[3].theta_dot

            phi = np.arctan2(ydot, xdot)
            #if ((phi>0.0) and (phi<RELEASE_POINT)):
            if (clock-arm.joint_execution_time[2])>arm.delta_ts[3]:
                if not arm.joint_state[3]:
                    # print('delta_t3', clock-arm.joint_execution_time[2])
                    arm.joint_state[3] = True
                    arm.joint_execution_time[3] = clock
                arm.release = True
                arm.mode = PD_CONTROL
