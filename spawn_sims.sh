#!/usr/bin/env bash
#0.2190, 0.0865, 0.02050
SIM_TIME=0.1
python3 sim.py --sim_id 0 --sim_time $SIM_TIME --delta_t1 0.2190 --delta_t2 0.0865 --delta_t3 0.02050 &
python3 sim.py --sim_id 1 --sim_time $SIM_TIME --delta_t1 0.390 --delta_t2 0.0865 --delta_t3 0.06050 &
python3 sim.py --sim_id 2 --sim_time $SIM_TIME --delta_t1 0.2190 --delta_t2 0.0865 --delta_t3 0.05050 &
python3 sim.py --sim_id 3 --sim_time $SIM_TIME --delta_t1 0.4190 --delta_t2 0.0865 --delta_t3 0.01050 &
python3 sim.py --sim_id 4 --sim_time $SIM_TIME --delta_t1 0.1900 --delta_t2 0.0865 --delta_t3 0.04050 &
python3 sim.py --sim_id 5 --sim_time $SIM_TIME --delta_t1 0.2090 --delta_t2 0.0865 --delta_t3 0.02050 &
python3 sim.py --sim_id 6 --sim_time $SIM_TIME --delta_t1 0.2590 --delta_t2 0.0865 --delta_t3 0.03050 &
