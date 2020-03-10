from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import os
from multiprocessing import Pool
from config import *

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

def flush_result(result_path):
    if os.path.exists(result_path):
        with open(result_path, 'r+') as f:
            f.truncate(0)
            f.close()
def read_result(result_path):
    result = {}
    with open(result_path, 'r') as f:
        for line in f:
            sim_id, distance = line.split(',')
            result[sim_id] = float(distance)
    return result
def sample_cost(seed_delta_ts, n_samples=3, sampling_radius = 0.01):
    delta_ts_samples = generate_petrubations(seed_delta_ts, n_samples, sampling_radius=sampling_radius)
    flush_result(SAMPLE_RESULT_PATH)
    processes = []

    for sim_id, delta_ts in enumerate(delta_ts_samples):
        processes.append([sim_id, SIM_TIME, DISPLAY_ON, delta_ts[0], delta_ts[1], delta_ts[2]])
    pool = Pool(processes=n_samples)
    pool.map(run_processes, processes)
    result_dict = read_result(SAMPLE_RESULT_PATH)
    flush_result(SAMPLE_RESULT_PATH)
    y = np.array([result_dict[str(i)] for i in range(n_samples)])
    X = np.array([processes[i][3:] for i in range(n_samples)])
    print('Sampling Done!!!')
    return X, y

def generate_petrubations(delta_ts, n_samples, sampling_radius=0.01):
    temp = delta_ts
    delta_ts += sampling_radius*np.random.randn(n_samples, len(delta_ts))
    delta_ts[delta_ts<0] = 0
    return delta_ts

def run_processes(process):
    os.system('python3 sim.py --sim_id {} --sim_time {} --display {} --delta_t1 {} --delta_t2 {} --delta_t3 {}'.format(*process))
