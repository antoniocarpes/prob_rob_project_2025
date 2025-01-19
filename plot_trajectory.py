import numpy as np
import math
import matplotlib.pyplot as plt
import pickle
import utils
from compute_trajectory import compute_trajectory



with open('dataset/ticks_transitions.pkl', 'rb') as f:
    ticks_trans = pickle.load(f)

for values in ticks_trans:
    if np.abs(values[1]) > 5000:
        values[1] = values[1]%5000

#print(max(ticks_trans, key=lambda x: x[1]))


X_guess = [1.5, 0, 0, 0.1, 0.0106141, 1.4, 0]

trajectory, trajectory_sensor = compute_trajectory(ticks_trans, X_guess)


x_np = np.asarray([values[0] for values in trajectory_sensor])
y_np = np.asarray([values[1] for values in trajectory_sensor])

fig = plt.figure()
ax = fig.add_subplot(111)
ax.scatter(x_np, y_np)
ax.axis("equal")
plt.show()