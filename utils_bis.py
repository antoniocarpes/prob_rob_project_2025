import numpy as np
import matplotlib.pyplot as plt

def v2t(pose):
    x, y, theta = pose
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, x],
                     [s,  c, y],
                     [0,  0, 1]])

def t2v(T):
    x, y = T[0, 2], T[1, 2]
    theta = np.arctan2(T[1, 0], T[0, 0])
    return np.array([x, y, theta])

def invert_isometry(T):
    R = T[:2, :2]
    t = T[:2, 2]
    T_inv = np.eye(3)
    T_inv[:2, :2] = R.T
    T_inv[:2, 2] = -R.T @ t
    return T_inv

def h_odom_robot(traction_tick, steer_tick, X, theta_prev):
    Ksteer, Ktraction, base_line, steer_offset, X_0, Y_0, Theta_0 = X
    steer_angle = steer_tick * Ksteer + steer_offset
    d = traction_tick * Ktraction

    if np.abs(np.tan(steer_angle)) < 1e-6:
        Delta_x = d * np.cos(theta_prev)
        Delta_y = d * np.sin(theta_prev)
        Delta_theta = 0.0
    else:
        R = base_line / np.tan(steer_angle)
        Delta_theta = d / R
        Delta_x = R * (np.sin(theta_prev + Delta_theta) - np.sin(theta_prev))
        Delta_y = -R * (np.cos(theta_prev + Delta_theta) - np.cos(theta_prev))

    delta_robot_pose = v2t([Delta_x, Delta_y, Delta_theta])
    sensor_to_robot = v2t([X_0, Y_0, Theta_0])
    delta_sensor_pose = invert_isometry(sensor_to_robot) @ delta_robot_pose @ sensor_to_robot
    return delta_sensor_pose

def compute_trajectory(traction_ticks, steer_ticks, X):
    n = len(traction_ticks)
    trajectory = [np.eye(3)]
    robot_theta = 0.0

    for i in range(1, n):
        traction_delta = traction_ticks[i] - traction_ticks[i-1]
        delta_pose = h_odom_robot(traction_delta, steer_ticks[i], X, robot_theta)
        new_pose = trajectory[-1] @ delta_pose
        trajectory.append(new_pose)

        sensor_to_robot = v2t([X[4], X[5], X[6]])
        robot_pose = np.dot(new_pose, invert_isometry(sensor_to_robot))
        robot_theta = t2v(robot_pose)[2]

    return trajectory

def error_jacobian(trajectory, delta_Z, X, traction_ticks, steer_ticks):
    e = []
    J = []
    eps = 1e-5

    for i in range(1, len(trajectory)):
        traction_delta = traction_ticks[i] - traction_ticks[i-1]
        pred = h_odom_robot(traction_delta, steer_ticks[i], X, t2v(trajectory[i-1])[2])
        err_mat = invert_isometry(delta_Z[i-1]) @ pred
        err = t2v(err_mat)
        e.extend(err)

        Ji = np.zeros((3, len(X)))
        for j in range(len(X)):
            X_eps_plus = X.copy()
            X_eps_plus[j] += eps
            X_eps_minus = X.copy()
            X_eps_minus[j] -= eps

            pred_plus = h_odom_robot(traction_delta, steer_ticks[i], X_eps_plus, t2v(trajectory[i-1])[2])
            pred_minus = h_odom_robot(traction_delta, steer_ticks[i], X_eps_minus, t2v(trajectory[i-1])[2])

            err_plus = t2v(invert_isometry(delta_Z[i-1]) @ pred_plus)
            err_minus = t2v(invert_isometry(delta_Z[i-1]) @ pred_minus)

            Ji[:, j] = (err_plus - err_minus) / (2 * eps)
        J.append(Ji)

    return np.array(e), np.vstack(J)

def ls_calibration(traction_ticks, steer_ticks, delta_Z, X_guess, max_iter=10):
    X = X_guess.copy()
    for _ in range(max_iter):
        trajectory = compute_trajectory(traction_ticks, steer_ticks, X)
        e, J = error_jacobian(trajectory, delta_Z, X, traction_ticks, steer_ticks)
        H = np.dot(J.T, J)
        b = -np.dot(J.T, e)
        dx = np.linalg.solve(H, b)
        X += dx
        if np.linalg.norm(dx) < 1e-6:
            break
    return X
