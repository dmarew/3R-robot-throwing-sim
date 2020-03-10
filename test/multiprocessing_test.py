import os
from multiprocessing import Pool
SIM_TIME=0.1
N = 2
processes = []
delta_t1 = 0.2190
delta_t2 = 0.0865
delta_t3 = 0.0205

for sim_id in range(N):
    processes.append([sim_id, SIM_TIME, delta_t1, delta_t2, delta_t3])

def run_processes(process):
    os.system('python3 sim.py --sim_id {} --sim_time {} --delta_t1 {} --delta_t2 {} --delta_t3 {}'.format(*process))
pool = Pool(processes=N)
pool.map(run_processes, processes)
