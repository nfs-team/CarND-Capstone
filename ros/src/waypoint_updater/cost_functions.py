from helpers import logistic, to_equation, differentiate
from constants import *
import numpy as np

# COST FUNCTIONS
def min_velocity_cost(a, max_v, T, trajectory):
    s = a
    s_dot = differentiate(s)
    v = to_equation(s_dot)
    all_vels = [v(float(T) / 500 * i) for i in range(500)]
    min_vel = min(all_vels)
    if min_vel < -0.001:
        return 1
    else:
        return 0

def max_velocity_cost(a, max_v, T, trajectory):
    s = a
    s_dot = differentiate(s)
    v = to_equation(s_dot)
    all_vels = [v(float(T) / 500 * i) for i in range(500)]
    max_vel = max(all_vels, key=abs)
    if abs(max_vel) > max_v + 0.1:
        return 1
    else:
        return 0

def efficiency_cost(a, max_v, T, trajectory):
    """
    Rewards high average speeds.
    """
    t = T
    s = a
    s = to_equation(s)
    avg_v = float(s(t)) / t
    return logistic(2*float(max_v - avg_v) / avg_v)

def total_efficiency_cost(a, max_v, T, trajectory):
    """
    Rewards high average speeds.
    """

    t = T
    dist = trajectory.end[0] - trajectory.start[0]
    t = t + (trajectory.full_distance - dist)/max_v
    avg_v = trajectory.full_distance / t

    return logistic(2*float(max_v - avg_v) / avg_v)

def total_accel_cost(a, max_v, T, trajectory):
    s = a
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    a = to_equation(s_d_dot)
    total_acc = 0
    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        acc = a(t)
        total_acc += abs(acc*dt)
    acc_per_second = total_acc / T
    
    return logistic(acc_per_second / EXPECTED_ACC_IN_ONE_SEC )
    
def max_accel_cost(a, max_v, T, trajectory):
    s = a
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    a = to_equation(s_d_dot)
    all_accs = [a(float(T)/500 * i) for i in range(500)]
    max_acc = max(all_accs, key=abs)
    if abs(max_acc) > MAX_ACCEL: return 1
    else: return 0
    

def max_jerk_cost(a, max_v, T, trajectory):
    s = a
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    jerk = differentiate(s_d_dot)
    jerk = to_equation(jerk)
    all_jerks = [jerk(float(T)/500 * i) for i in range(500)]
    max_jerk = max(all_jerks, key=abs)
    if abs(max_jerk) > MAX_JERK: return 1
    else: return 0

def total_jerk_cost(a, max_v, T, trajectory):
    s = a
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    jerk = to_equation(differentiate(s_d_dot))
    total_jerk = 0
    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        j = jerk(t)
        total_jerk += abs(j*dt)
    jerk_per_second = total_jerk / T
    return logistic(jerk_per_second / EXPECTED_JERK_IN_ONE_SEC )