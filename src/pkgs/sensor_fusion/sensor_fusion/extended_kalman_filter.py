"""
Extended kalman filter (EKF) localization sample
author: Atsushi Sakai (@Atsushi_twi)
edited by: Peter Nguyen
"""

import math

import numpy as np

# Variance values not yet measured

# Covariance matrix of process noise
Q = np.diag([
    0.2,    # variance of location on x-axis
    0.2,    # variance of location on y-axis
    0.05,   # variance of yaw angle
    1.5     # variance of velocity
]) ** 2     # predict state covariance

# GPS NEO-M9N 1.5m accuracy
# Covariance matrix of observation noise: x,y position covariance
R = np.diag([1.5, 1.5]) ** 2


def motion_model(x, u, DT):
    F = np.array([[1.0, 0,   0,   0],
                  [0,   1.0, 0,   0],
                  [0,   0,   1.0, 0],
                  [0,   0,   0,   0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0,                    DT],
                  [1.0,                    0.0]])

    x = F @ x + B @ u

    return x


def observation_model(x):
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    z = H @ x

    return z


def jacob_f(x, u, DT):
    """
    Jacobian of Motion Model
    motion model
    x_{t+1} = x_t+v*dt*cos(yaw)
    y_{t+1} = y_t+v*dt*sin(yaw)
    yaw_{t+1} = yaw_t+omega*dt
    v_{t+1} = v{t}
    so
    dx/dyaw = -v*dt*sin(yaw)
    dx/dv = dt*cos(yaw)
    dy/dyaw = v*dt*cos(yaw)
    dy/dv = dt*sin(yaw)
    """
    yaw = x[2, 0]
    v = u[0, 0]
    jF = np.array([
        [1.0, 0.0, -DT * v * math.sin(yaw), DT * math.cos(yaw)],
        [0.0, 1.0,  DT * v * math.cos(yaw), DT * math.sin(yaw)],
        [0.0, 0.0,  1.0,                    0.0],
        [0.0, 0.0,  0.0,                    1.0]])

    return jF


def jacob_h():
    # Jacobian of Observation Model
    jH = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    return jH


def ekf_estimation(xEst, PEst, z, u, DT):
    #  Predict
    xPred = motion_model(xEst, u, DT)  # Predict state vector values
    jF = jacob_f(xEst, u, DT)  # Jacobian of Motion Model
    PPred = jF @ PEst @ jF.T + Q  # Predict covariance matrix values

    #  Update
    jH = jacob_h()  # Jacobian of Observation Model
    zPred = observation_model(xPred)  # Observation vector prediction
    y = z - zPred  # Output signal
    S = jH @ PPred @ jH.T + R                   #
    K = PPred @ jH.T @ np.linalg.inv(S)         #
    xEst = xPred + K @ y  # EKF state estimation
    PEst = (np.eye(len(xEst)) - K @ jH) @ PPred  # EKF covariance estimation
    return xEst, PEst
