import numpy as np
import matplotlib.pyplot as plt
from utils_bis import *
if __name__=='__main__':

    with open('dataset/ticks_subsampled.txt') as file:
        delta_Z = []     # isometrías relativas 3x3
        tick_trans = []
        for line in file:
            ticks = list(line.strip().split())[0:2]
            delta_pose = list(map(float, line.strip().split()[2:5]))  # [x, y, theta]
            delta_Z.append(v2t(delta_pose))  # convert to 3x3 matrix
            tick_trans.append(list(map(float, ticks)))
    traction_ticks = [traction for _ ,traction in tick_trans]
    steer_ticks = [steer for steer ,_ in tick_trans]

    initial_pose = v2t(np.array([0,0,0]))
    ground_truth = [np.dot(initial_pose, delta_Z[0])]
    for pose in delta_Z[1:]:
        next_pose = np.dot(ground_truth[-1], pose)
        ground_truth.append(next_pose)
    
    x_guess = []
    y_guess = []
    for i, pose in enumerate(ground_truth):
        pose_vector = t2v(pose)
        x_guess.append(float(pose_vector[0]))
        y_guess.append(float(pose_vector[1]))
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.scatter(x_guess, y_guess)
    ax.axis("equal")
    plt.show()
    

    X_guess = np.array([0.1, 0.0106141, 1.4, 0, 1.5, 0, 0])  # Ksteer, Ktraction, base_line, steer_offset, X0, Y0, Theta0
    X_opt = ls_calibration(traction_ticks, steer_ticks, delta_Z, X_guess)
    print("Parámetros calibrados:", X_opt)

    trajectory = compute_trajectory(traction_ticks, steer_ticks, X_opt)
    trajectory = [t2v(pose) for pose in trajectory]
 
    x_guess = []
    y_guess = []
    for i, pose in enumerate(trajectory):
        pose_vector = pose
        x_guess.append(float(pose_vector[0]))
        y_guess.append(float(pose_vector[1]))
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.scatter(x_guess, y_guess)
    ax.axis("equal")
    plt.show()