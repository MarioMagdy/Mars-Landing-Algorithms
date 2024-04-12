import numpy as np

initial_horizontal_velocity = 10  # m/s (positive to the right)
initial_vertical_velocity = 444.0 
x_velocity = initial_horizontal_velocity
y_velocity = initial_vertical_velocity
vel_vector = np.array([x_velocity, y_velocity])
class IMU:
    def __init__(self, lander):
        self.lander = lander
        self.current_acceleration = np.array([0.0, 0.0])
        self.last_velocity = np.array([0.0, 0.0])
        


    def fixed_update(self, delta_time):
        # Accelometer Sensor code & Gyroscope Sensor Code calculated from change rate of velocity vector & change rate of Euler Angles
        self.current_acceleration = (vel_vector - self.last_velocity) / delta_time
        self.last_velocity = vel_vector

#calling function

my_imu = IMU("Mars Lander")

delta_time = 3 

my_imu.fixed_update(delta_time)





def update_orientation(dt):
    global spacecraft_orientation
    global max_thrust
    global left_wing_up_thruster_power, left_wing_down_thruster_power
    global right_wing_up_thruster_power, right_wing_down_thruster_power
    global top_left_thruster_power, top_right_thruster_power
    global bottom_left_thruster_power, bottom_right_thruster_power

    # Calculate the net force differences for rotation
    net_vertical_force_difference = (right_wing_up_thruster_power - right_wing_down_thruster_power) - \
                                    (left_wing_up_thruster_power - left_wing_down_thruster_power)
    net_horizontal_force_difference = (bottom_right_thruster_power - bottom_left_thruster_power) - \
                                      (top_right_thruster_power - top_left_thruster_power)

    # Calculate the orientation change based on the net force differences
    # The multipliers can be adjusted to represent the sensitivity of the spacecraft's rotation
    orientation_change = (net_vertical_force_difference + net_horizontal_force_difference) * dt * 0.05  # Arbitrary value for rotation sensitivity

    # Update the spacecraft orientation based on the combined orientation changes
    spacecraft_orientation += orientation_change # SO THIS IS THE BANK ANGEL THEN????

# Define initial spacecraft orientation and thruster powers
spacecraft_orientation = 2 # Initial orientation in degrees
max_thrust = 100.0  # Maximum thrust value
left_wing_up_thruster_power = 100
left_wing_down_thruster_power = 40.0
right_wing_up_thruster_power = 45.0
right_wing_down_thruster_power = 35.0
top_left_thruster_power = 30.0
top_right_thruster_power = 25.0
bottom_left_thruster_power = 20.0
bottom_right_thruster_power = 30

# Define the time step
dt = 0.1  # Time step in seconds

# Call the update_orientation function with the time step
update_orientation(dt)

# Print the updated spacecraft orientation







# Example of readings
accel_data = np.array([my_imu.current_acceleration])
gyro_data = np.radians(np.array([spacecraft_orientation]))

# Constants
G = 3.44  # Acceleration due to gravity (m/s^2)
dt = 0.1  # Time step (seconds)

# We assume gyro initial with zero in 2D space.
velocity = np.zeros(2)  # 2D velocity (vx, vy)
position = np.zeros(2)  # 2D position (x, y)
attitude = np.zeros(1)  # Euler angle (yaw in 2D)

# Complementary filter coefficients
alpha = 0.98  # Weight for accelerometer data
beta = 1 - alpha  # Weight for gyroscope data


def complementary_filter(accel_data, gyro_data, alpha, beta):
    # Fuse accelerometer and gyroscope data using complementary filter
    accel_filtered = alpha * accel_data
    gyro_integration = np.cumsum(gyro_data, axis=0) * dt
    gyro_filtered = beta * gyro_integration
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
for i in range(1, len(accel_data)):  # Assuming data is synchronized
    # Sensor fusion using complementary filter
    fused_data = complementary_filter(accel_data[i], gyro_data[i], alpha, beta)

    # Integration using Runge-Kutta method
    velocity, position = integrate_rk4(fused_data, velocity, position)

    # Update attitude estimation (for 2D, yaw angle is updated)
    attitude += gyro_data[i][2] * dt  # Update yaw angle
    
    print("Body Velocity:", velocity)
    print("Body Position:", position)
    print("Body Attitude (Yaw angle):", np.degrees(attitude))



