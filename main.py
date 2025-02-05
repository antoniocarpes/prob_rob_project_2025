import numpy as np
import pickle
import utils
import matplotlib.pyplot as plt

with open('dataset/tick_transitions.pkl', 'rb') as f:
    tick_trans = pickle.load(f)

with open('dataset/ground_truth.pkl', 'rb') as f:
    ground_truth = pickle.load(f)

#convert both lists values to float as they were stored as strings
tick_trans = [[float(value) for value in sublist] for sublist in tick_trans]
ground_truth = [[float(value) for value in sublist] for sublist in ground_truth]

#let us plot both ground_truth trajectories and predicted trajectory with h_odom

#ground_truth
x = []
y = []
for i in range(len(ground_truth)):
    x.append(float(ground_truth[i][0]))
    y.append(float(ground_truth[i][1]))
fig = plt.figure()
ax = fig.add_subplot(111)
ax.scatter(x, y)
ax.axis("equal")
plt.show()

#prediction with h_odom
X_guess = [1.5, 0, 0, 0.1, 0.0106141, 1.4, 0]
trajectory_sensor = utils.compute_trajectory(X_guess, tick_trans)
#trajectory_sensor = utils.compute_trajectory(tick_trans, X_guess)

x_guess = []
y_guess = []
for i, pose in enumerate(trajectory_sensor):
    pose_vector = utils.t2v(pose)
    x_guess.append(float(pose_vector[0]))
    y_guess.append(float(pose_vector[1]))
fig = plt.figure()
ax = fig.add_subplot(111)
ax.scatter(x_guess, y_guess)
ax.axis("equal")
plt.show()

#now that we have ground truth and predicted trajectory, compute error and Jacobian and apply Least Squares method

#X_sol = utils.ls_calibration(trajectory_sensor,ground_truth, X_guess, tick_trans)
#print(type(ground_truth[0][0]))

#print(X_sol)
