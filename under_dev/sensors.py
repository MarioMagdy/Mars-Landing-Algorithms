import numpy as np

initial_horizontal_velocity = 6000.0  # m/s (positive to the right)
initial_vertical_velocity = -5333.0 
x_velocity = initial_horizontal_velocity
y_velocity = initial_vertical_velocity
vel_vector = np.array([x_velocity, y_velocity])
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
print("Updated spacecraft orientation:", spacecraft_orientation)


class IMU:
    def __init__(self, lander):
        self.lander = lander
        self.current_acceleration = np.array([0.0, 0.0])
        self.last_velocity = np.array([0.0, 0.0])
        self.last_euler_angle = np.array([0.0, 0.0])
        self.current_change_rate_of_euler_angle = np.array([0.0, 0.0])

    def start(self):
        # Assuming 'get_euler_angles' is a method that retrieves the Euler angles
        self.last_euler_angle = self.spacecraft_orientation()

    def fixed_update(self, delta_time):
        # Accelometer Sensor code & Gyroscope Sensor Code calculated from change rate of velocity vector & change rate of Euler Angles
        current_euler_angle = self.spacecraft_orientation()
        self.current_acceleration = (vel_vector - self.last_velocity) / delta_time
        self.current_change_rate_of_euler_angle = (current_euler_angle - self.last_euler_angle) / delta_time
        self.last_velocity = vel_vector
        self.last_euler_angle = current_euler_angle

    def spacecraft_orientation(self):
        # Placeholder for getting Euler angles
        return np.array([0.0, 0.0])  # Dummy values for demonstration

my_imu = IMU("Mars Lander")

my_imu.start()

delta_time = 3 

my_imu.fixed_update(delta_time)

print("Current Acceleration:", my_imu.current_acceleration)

my_imu.fixed_update(delta_time)

print("current_change_rate_of_euler_angle", my_imu.current_change_rate_of_euler_angle)