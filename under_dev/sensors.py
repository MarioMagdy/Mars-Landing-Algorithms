import numpy as np

initial_horizontal_velocity = 6000.0  # m/s (positive to the right)
initial_vertical_velocity = -5333.0 
x_velocity = initial_horizontal_velocity
y_velocity = initial_vertical_velocity
vel_vector = np.array([x_velocity, y_velocity])

class IMU:
    def __init__(self, lander):
        self.lander = lander
        self.current_acceleration = np.array([0.0, 0.0])
        self.last_velocity = np.array([0.0, 0.0])
        self.last_euler_angle = np.array([0.0, 0.0])
        self.current_change_rate_of_euler_angle = np.array([0.0, 0.0])

    def start(self):
        # Assuming 'get_euler_angles' is a method that retrieves the Euler angles
        self.last_euler_angle = self.get_euler_angles()

    def fixed_update(self, delta_time):
        # Accelometer Sensor code & Gyroscope Sensor Code calculated from change rate of velocity vector & change rate of Euler Angles
        current_euler_angle = self.get_euler_angles()
        self.current_acceleration = (vel_vector - self.last_velocity) / delta_time
        self.current_change_rate_of_euler_angle = (current_euler_angle - self.last_euler_angle) / delta_time
        self.last_velocity = vel_vector
        self.last_euler_angle = current_euler_angle

    def get_euler_angles(self):
        # Placeholder for getting Euler angles
        return np.array([0.0, 0.0])  # Dummy values for demonstration

my_imu = IMU("Mars Lander")

my_imu.start()

delta_time = 3 

my_imu.fixed_update(delta_time)

print("Current Acceleration:", my_imu.current_acceleration)

my_imu.fixed_update(delta_time)

print("current_change_rate_of_euler_angle", my_imu.current_change_rate_of_euler_angle)