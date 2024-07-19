import numpy as np

class NavigationSystem:
    def __init__(self, accel_data, gyro_data, dt=0.1, alpha=0.98):
        self.accel_data = accel_data
        self.gyro_data = gyro_data
        self.dt = dt
        self.alpha = alpha
        self.beta = 1 - alpha

        # Constants
        self.G = 3.44  # Acceleration due to gravity (m/s^2)

        # Initial conditions
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)
        self.attitude = np.zeros(3)  # Euler angles (roll, pitch, yaw)

    def complementary_filter(self, accel_data, gyro_data):
        accel_filtered = np.array([self.alpha * ax for ax in accel_data])
        gyro_integration = np.cumsum(gyro_data, axis=0) * self.dt
        gyro_filtered = np.array([self.beta * gx for gx in gyro_integration])
        fused_data = accel_filtered + gyro_filtered
        return fused_data

    def integrate_rk4(self, fused_data):
        k1v = fused_data * self.dt
        k2v = (fused_data + 0.5 * k1v) * self.dt
        k3v = (fused_data + 0.5 * k2v) * self.dt
        k4v = (fused_data + k3v) * self.dt
        self.velocity += (k1v + 2*k2v + 2*k3v + k4v) / 6

        k1p = self.velocity * self.dt
        k2p = (self.velocity + 0.5 * k1p) * self.dt
        k3p = (self.velocity + 0.5 * k2p) * self.dt
        k4p = (self.velocity + k3p) * self.dt
        self.position += (k1p + 2*k2p + 2*k3p + k4p) / 6

    def calculate_bank_angle(self, accel_data):
        accelX, accelY, accelZ = accel_data[0], accel_data[1], accel_data[2]
        bank_angle = np.arctan2(accelX, np.sqrt(accelY**2 + accelZ**2))
        return np.degrees(bank_angle)  # Convert the angle to degrees

    def run(self):
        for i in range(1, len(self.accel_data)):  # Assuming data is synchronized
            # Sensor fusion using complementary filter
            fused_data = self.complementary_filter(self.accel_data[i], self.gyro_data[i])
            # Integration using Runge-Kutta method
            self.integrate_rk4(fused_data)
            # Update attitude estimation (for illustration purposes, you'll need actual orientation estimation logic)
            bank_angle = self.calculate_bank_angle(self.accel_data[i])
            print(f"Bank: {bank_angle} degrees")

        # Calculate flight path angle
        fpa_rad = np.arctan2(self.velocity[1], self.velocity[0])
        print(f"Flight Path Angle (FPA) in radians: {fpa_rad}")

    def euler_to_rotation_matrix(self, euler_angles):
        roll, pitch, yaw = euler_angles
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])
        
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
        
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])
        
        R = np.dot(R_z, np.dot(R_y, R_x))
        return R

    def transform_to_inertial_frame(self):
        rotation_matrix = self.euler_to_rotation_matrix(self.attitude)
        accel_inertial = np.dot(rotation_matrix, self.accel_data.T).T
        gyro_inertial = np.dot(rotation_matrix, self.gyro_data.T).T
        return accel_inertial, gyro_inertial



