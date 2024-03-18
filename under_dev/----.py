import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as UKF

# Define state transition function for position and attitude estimation
def f(x,dt):
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
# Initialize the Unscented Kalman Filter
ukf = UKF(dim_x=13, dim_z=6, fx=fx, hx=hx)

# Initialize state and covariance matrices
ukf.x = np.zeros(13)  # Initial state estimate for position (x, y, z), velocity, and attitude (quaternions)
ukf.P = np.eye(13)    # Initial state covariance
ukf.R = np.eye(6)     # Measurement noise covariance for gyro and accelerometer data
ukf.Q = np.eye(13)    # Process noise covariance

# Main loop
for data in sensor_data:
    gyro_data, accel_data = data[:3], data[3:]  # Extract gyro and accelerometer readings in 3 axes
    dt = 0.1# Time step between sensor readings
    
    # Predict step
    ukf.predict(dt=dt)
    
    # Update step with gyro and accelerometer data
    measurement = np.concatenate((gyro_data, accel_data))
    ukf.update(measurement)
    
    # Estimate position and attitude
    position_estimate = ukf.x[:3]  # Extract position estimate in 3 axes
    attitude_estimate = ukf.x[6:10]  # Extract attitude estimate (quaternions)
    
    print("Estimated Position (x, y, z):", position_estimate)
    print("Estimated Attitude (quaternions):", attitude_estimate)