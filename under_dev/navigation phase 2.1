import numpy as np

#Example of readings
accel_data = np.array([[-0.1147, -0.1304, 0.9873], [-0.12, -0.42, 0.95], [-0.5, 0, 0.60]])
gyro_data = np.radians(np.array([[85, 90, 43], [20, 40, 40],[10, 0, 40]]))


# Constants
G = 3.44  # Acceleration due to gravity (m/s^2)
dt = 0.1  # Time step (seconds)

#We assume gyro and acceleration inital with zero.
velocity = np.zeros(3)
position = np.zeros(3)
attitude = np.zeros(3)  # Euler angles (roll, pitch, yaw)

# Complementary filter coefficients
alpha = 0.98  # Weight for accelerometer data
beta = 1 - alpha  # Weight for gyroscope data


def complementary_filter(accel_data, gyro_data, alpha, beta):
    # Fuse accelerometer and gyroscope data using complementary filter
    accel_filtered = np.array([alpha * ax for ax in accel_data])
    gyro_integration = np.cumsum(gyro_data, axis=0) * dt
    gyro_filtered = np.array([beta * gx for gx in gyro_integration])
    fused_data = accel_filtered + gyro_filtered
    return fused_data

def integrate_rk4(fused_data, velocity, position):
    # Runge-Kutta 4th order integration to calculate velocity and position
    k1v = fused_data * dt
    k2v = (fused_data + 0.5 * k1v) * dt
    k3v = (fused_data + 0.5 * k2v) * dt
    k4v = (fused_data + k3v) * dt
    velocity += (k1v + 2*k2v + 2*k3v + k4v) / 6

    k1p = velocity * dt
    k2p = (velocity + 0.5 * k1p) * dt
    k3p = (velocity + 0.5 * k2p) * dt
    k4p = (velocity + k3p) * dt
    position += (k1p + 2*k2p + 2*k3p + k4p) / 6

    return velocity, position

# Main loop for estimation
for i in range(1,len(accel_data)):  # Assuming data is synchronized
    # Sensor fusion using complementary filter
    fused_data = complementary_filter(accel_data[i], gyro_data[i], alpha, beta)

    # Integration using Runge-Kutta method
    velocity, position = integrate_rk4(fused_data, velocity, position)

    # Update attitude estimation (for illustration purposes, you'll need actual orientation estimation logic)
    attitude += np.array(gyro_data[i]) * dt  # Update each axis separately
    print("body Velocity:", velocity)
    print("body Position:", position)
    print("body Attitude (Euler angles - roll, pitch, yaw):", attitude)
   
#from body frame to intial frame
def euler_to_rotation_matrix(euler_angles):
    # Convert Euler angles (roll, pitch, yaw) to rotation matrix
    roll, pitch, yaw = euler_angles
    cy = np.cos(yaw)
    sy = np.sin(yaw)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cr = np.cos(roll)
    sr = np.sin(roll)

    rotation_matrix = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ])

    return rotation_matrix

# Calculate the bank angle (roll) using the accelerometer data
def calculate_bank_angle(accel_data):
    # Assuming you have accelerometer readings: accelX =accel_data[0], accelY =accel_data[1], accelZ=accel_data[2]
    
    bank_angle = np.arctan2(accel_data[1], np.sqrt(accel_data[0]**2 + accel_data[2]**2))
    
    return np.degrees(bank_angle)  # Convert the angle to degrees


# Convert attitude to rotation matrix
rotation_matrix = euler_to_rotation_matrix(attitude)

# Transform accelerometer data from body frame to inertial frame
accel_inertial = np.dot(rotation_matrix, accel_data)

# Transform gyroscope data from body frame to inertial frame
gyro_inertial = np.dot(rotation_matrix, gyro_data)

#from body to intial
print("Accelerometer data in inertial frame:", accel_inertial)
print("Gyroscope data in inertial frame:", gyro_inertial)




bank_angle = calculate_bank_angle(accel_data)
print(f"Bank Angle: {bank_angle} degrees")
