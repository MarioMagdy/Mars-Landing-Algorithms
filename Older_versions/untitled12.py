import numpy as np
from scipy.spatial.transform import Rotation as R

 # Constants
mass = 1000  # kg
g = 3.71  # m/s^2 (Martian gravity)
kp = 1  # Proportional control gain
kd = 0.1  # Derivative control gain
drag_coeff = 0.5
dt = 0.01  # (seconds) time steps
t_max = 10  # (seconds) total simulation time
density = 0.02 #kg/m³

# Thruster configuration 
thruster_directions = np.array([[1, 0, 0],
                               [-1, 0, 0],
                               [0, 1, 0],
                               [0, -1, 0],
                               [0, 0, 1],
                               [0, 0, -1]])


class MarsLanderControlSystem:
    def __init__(self):
            # Initialize sensor data
        self.accelerometer = [0, 0, -9.81]  # m/s^2 
        self.gyroscope = [0, 0, 0]          # rad/s 
        self.desired_orientation = [0, 0, 0] 
    
        
    def update_sensors(self, acceleration, rotation):
        self.accelerometer = acceleration
        self.gyroscope = rotation
            
    def generate_sensor_data(time):
        position = np.array([1000, 500, 200]) - 100 * time
        velocity = -100 * np.ones(3)
        return position, velocity
    def compute_bank_angle(self, velocity):
        bank_angle = np.arctan(velocity / 10)  
        return bank_angle
    
    def compute_control_command(self, velocity):
        # Compute error between desired and current orientation
        error = np.array(self.desired_orientation) - np.array(self.gyroscope)
    
        # Integral error
        self.integral_error += error
    
        # Derivative error
        derivative_error = error - self.prev_error
        self.prev_error = error
    
        # Compute bank angle based on velocity
        bank_angle = self.compute_bank_angle(velocity)
        control_command = (self.Kp * error +
                           self.Ki * self.integral_error +
                           self.Kd * derivative_error +
                           [0, 0, bank_angle])

    
    def calculate_drag(self, velocity):
        return -drag_coeff * np.linalg.norm(velocity) * velocity
        
    def calculate_feedforward_signal(self, desired_state, current_state, time):
        predicted_velocity = desired_state[3:6]  # Assuming desired velocity is known
        predicted_drag = self.calculate_drag(predicted_velocity, density)
        feedforward_force = -predicted_drag
        return feedforward_force
    def sensor_fusion(self, gyroscope_data, accelerometer_data, state):
        # Implement Kalman filter or other sensor fusion algorithm
        # to estimate orientation and acceleration from sensor data
        # Update state vector based on the estimated values
        return self.updated_state
        
    def control_loop(self, time):
        gyroscope_data, accelerometer_data = self.generate_sensor_data(time)
        state = self.sensor_fusion(gyroscope_data, accelerometer_data, time)
    
        # Convert quaternion to rotation matrix for calculations
        orientation_quat = state[6:10]
        orientation_matrix = R.from_quat(orientation_quat).as_matrix()
    
        # Desired state (replace with actual guidance calculations)
        desired_position = np.zeros(3)
        desired_velocity = np.zeros(3)
    
        # Error calculations
        position_error = desired_position - state[0:3]
        velocity_error = desired_velocity - state[3:6]
    
        # Control law with PID and feedforward
        control_force = -kp * position_error - kd * velocity_error + self.calculate_feedforward_signal(desired_state=desired_position, current_state=state, time=time)
    
        # Apply control force in thruster directions
        total_thrust = np.linalg.norm(control_force)
        thruster_forces = total_thrust * np.dot(orientation_matrix.T, thruster_directions)
    
        # Update state using simplified dynamics
        acceleration = (control_force + g * np.array([0, 0, 1])) / mass
        state[3:6] += acceleration * dt
        state[0:3] += state[3:6] * dt
        
    
    # Simulation loop
    time = np.arange(0, t_max, dt)
    state = np.zeros((len(time), 10))
    state[0] = np.array([1000, 500, 200, 0, 0, 0, 1, 0, 0, 0])  # Initial state
        
    for i in range(1, len(time)):
        control_loop()
        state[i] = state[i-1]
        
    
    # Print desired parameters
    print("Final position:", state[-1, 0:3])
    print("Final velocity:", state[-1, 3:6])
    print("Final orientation:", state[-1, 6:10]) 
    print("Final acceleration:", (state[-1, 3:6] - state[-2, 3:6]) / dt)  



