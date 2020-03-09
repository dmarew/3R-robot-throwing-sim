from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
def plot_spline(sim_id):
    with open('results/sim_{}_spline.txt'.format(sim_id), 'r') as f:
        spline = []
        for line in f :
            spline.append([float(i) for i in line.split(',')])
    spline = np.array(spline)
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    ax.scatter3D(spline[:, 0], spline[:, 1], spline[:, 2], 'gray')
    ax.set_xlabel('Theta1')
    ax.set_ylabel('Theta2')
    ax.set_zlabel('Theta3')
    fig = plt.figure()
    plt.plot(spline[:, 0], spline[:, 3])
    plt.xlabel('Theta1')
    plt.ylabel('Theta1_dot')
    plt.show()
def plot_ball_traj(sim_id):
    with open('results/sim_{}_ball_pos.txt'.format(sim_id), 'r') as f:
        traj = []
        for line in f :
            traj.append([float(i) for i in line.split(',')])
    traj = np.array(traj)
    fig = plt.figure()
    plt.plot(traj[:, 0], traj[:, 1])
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()
