import numpy as np
import math
import matplotlib.pyplot as plt
import pickle
import utils



def compute_trajectory(ticks_trans, X_guess):
    x_prev = 0.0
    y_prev = 0.0
    theta_prev = 0.0
    trajectory = [[0.0, 0.0, 0.0]]
    sensor_pose = [X_guess[0], X_guess[1], X_guess[2]]
    x_sensor = X_guess[0]
    y_sensor = X_guess[1]
    theta_sensor = X_guess[2]
    k_s = X_guess[3]
    k_tr = X_guess[4]
    d = X_guess[5]
    alpha_0 = X_guess[6]
    t_s = [value[0] for value in ticks_trans]
    t_tr = [value[1] for value in ticks_trans]
    for i in range(len(ticks_trans)):
        x_curr = x_prev + k_tr * t_tr[i]*math.cos(k_s*t_s[i]-alpha_0)*math.cos(theta_prev)
        y_curr = y_prev + k_tr * t_tr[i]*math.cos(k_s*t_s[i]-alpha_0)*math.sin(theta_prev)
        theta_curr = theta_prev + k_tr*t_tr[i]/d * math.sin(k_s*t_s[i]-alpha_0)
        trajectory.append([x_curr, y_curr, theta_curr])

        x_prev = x_curr
        y_prev = y_curr
        theta_prev = theta_curr

        #now that we have the trajectory of the kinematic center, compute the trajectory of the 
        #sensor by transforming to the sensor system
    #print(utils.v2t(sensor_pose))
    trajectory_sensor = [utils.t2v(np.dot(utils.v2t(robot_pose), utils.v2t(sensor_pose))) for robot_pose in trajectory]

    
    return trajectory, trajectory_sensor


   

