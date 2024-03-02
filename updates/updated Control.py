import numpy as np

# Constants
mass = 1000  # kg
g_mars = 3.71  # m/s^2 (Martian gravity)
drag_coeff = 0.5
density = 0.02  # kg/m³
dt = 0.01  # (seconds) time step
t_max = 10  # (seconds) total simulation time

class MarsLander:
    """
    Class representing a Mars lander with simplified dynamics and state estimation.
    """

    def __init__(self):
        """
        Initializes the lander with state and sensor data.
        """
        # State (position, velocity, orientation, angular velocity)
        self.position = np.array([1000, 500, 200])  # m
        self.velocity = np.array([-100, -100, -100])  # m/s
        self.orientation = np.array([1, 0, 0, 0])  # Quaternion (initially upright)
        self.angular_velocity = np.zeros(3)  # rad/s

        # Sensor readings
        self.accelerometer = np.array([0, 0, -3.71])  # m/s^2 (Martian gravity)
        self.gyroscope = np.zeros(3)  # rad/s

        # Control parameters (replace with actual control design)
        self.kp = 1  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.1  # Derivative gain

        # State estimation parameters (replace with appropriate algorithm)
        self.estimated_orientation = self.orientation.copy()
        self.prev_error = np.zeros(4)  # Initialize previous error as a quaternion

    def update_state(self, dt):
        """
        Updates the lander's state using simplified equations of motion.
        """
        # Assuming constant gravity and neglecting drag for simplicity
        acceleration = self.accelerometer  # m/s^2
        self.velocity += acceleration * dt
        self.position += self.velocity * dt

        # Assuming sensor readings directly reflect state
        self.gyroscope = self.angular_velocity

    def compute_bank_angle(self, desired_orientation):
        """
        Calculates the desired bank angle using a simplified PID controller.
        """
        # Compare complete quaternions for accurate error calculation
        error = desired_orientation - self.estimated_orientation

        # PID control (replace with more sophisticated control logic)
        control_signal = self.kp * error + self.ki * self.integrate_error(error, dt) + self.kd * self.differentiate_error(error, dt)

        # Extract desired bank angle from the control signal (replace with appropriate mapping)
        bank_angle = control_signal[2]

        return bank_angle

    def integrate_error(self, error, dt):
        # Now correctly accumulates error as a quaternion
        self.prev_error = self.prev_error + error  # Update previous error quaternion
        return self.ki * self.prev_error  # Return integrated error as a quaternion

    def differentiate_error(self, error, dt):
        # Consider using the full error quaternion for control calculation
        # Replace with the appropriate calculation based on your control strategy
        derivative_error = (error - self.prev_error) / dt
        self.prev_error = error  # Update previous error (all four components)
        return derivative_error

    def apply_thrusters(self, bank_angle):
        # Apply thruster forces based on the bank angle
        thruster_forces = np.array([
            [0, 0, 0],  # Front thrusters
            [0, 0, 0],  # Rear thrusters
            [0, 0, 0],  # Left thrusters
            [0, 0, 0],  # Right thrusters
        ])

        # Update acceleration based on thruster forces
        total_thrust = np.linalg.norm(thruster_forces)
        acceleration_due_to_thrusters = total_thrust / mass
        self.acceleration += acceleration_due_to_thrusters

    def update_orientation(self, bank_angle):
        # Update orientation based on the bank angle
        # Here we assume a simple model: directly updating the orientation quaternion
        delta_quaternion = np.array([np.cos(bank_angle / 2), 0, 0, np.sin(bank_angle / 2)])
        self.orientation = np.quaternion(*delta_quaternion) * np.quaternion(*self.orientation)

    def simulate_descent(self, dt, t_max):
        """
        Simulates the lander's descent in a loop.
        """
        time = np.arange(0, t_max, dt)
        for t in time:
            # Control logic: determine desired orientation
            desired_orientation = np.array([1, 0, 0, 0])  # Quaternion for upright

            # Calculate bank angle
            bank_angle = self.compute_bank_angle(desired_orientation)

            # Apply thrusters
            self.apply_thrusters(bank_angle)

            # Update estimated orientation
            self.update_orientation(bank_angle)

            # Update state
            self.update_state(dt)

            # Print state for debugging
            print("Time:", t)
            print("Position:", self.position)
            print("Velocity:", self.velocity)
            print("Orientation:", self.orientation)
            print("Accelerometer:", self.accelerometer)
            print("---")




