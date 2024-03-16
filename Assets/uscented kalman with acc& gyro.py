import numpy as np
from pykalman import UnscentedKalmanFilter

def fuse_accel_gyro(acc, gyro, dt=0.3):
 
  gyro = np.deg2rad(gyro)
# define the state transition function
  def f(x):
    # x is the state vector of shape (6,)
    # x[0:3] are the roll, pitch, and yaw angles in radians
    # x[3:6] are the angular velocities in radians/s
 
    x[0] += x[3] * dt
    x[1] += x[4] * dt
    x[2] += x[5] * dt

    return x

  # define the observation function
  def h(x):

    # calculate the acceleration components from the angles
    ax = -np.sin(x[1])
    ay = np.sin(x[0]) * np.cos(x[1])
    az = np.cos(x[0]) * np.cos(x[1])

    return np.array([ax, ay, az])


  x = np.zeros(6)

  # initialize the state covariance matrix with small values
  P = np.eye(6) * 0.01

  # initialize the measurement noise covariance matrix
  R = np.eye(3) * 0.01

  # initialize the process noise covariance matrix
  Q = np.eye(6) * 0.001

  # initialize the unscented Kalman filter
  ukf = UnscentedKalmanFilter(transition_functions=f, observation_functions=h, transition_covariance=Q, observation_covariance=R, initial_state_mean=x, initial_state_covariance=P)

  # for storing roll, pitch, and yaw angles
  rpy = []

  # loop through the accelerometer and gyroscope readings
  for i in range(len(acc)):
    # predict the next state


    # append the angles to the list
    rpy.append(x[0:3])

  # convert the angles to degrees
  rpy = np.rad2deg(rpy)

  # return the roll, pitch, and yaw angles as a numpy array
  rpy = np.array(rpy)
  return rpy


acc = np.array([[0.3, 0.5, 9.8], [0.2, 0.3, 9.7]])  # Replace with actual data
gyro = np.array([[0.01, 0.02, 0.03], [0.02, 0.03, 0.04]])  # Replace with actual data

fused_position = fuse_accel_gyro(acc, gyro)
print("Fused position:", fused_position)