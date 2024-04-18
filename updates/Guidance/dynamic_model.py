import numpy as np

class MarsEntry:
    """
    This class represents a simplified Mars entry dynamic model and calc next state **call the dynamics model method**
    """

    def __init__(self, mass, dt=0.1):
        self.mass = mass
        self.dt = dt

        # Constants
        self.G = 6.6743e-11  # Gravitational constant
        self.R_mars = 3389e3  # Mars radius
        

        

    def rho(self, z):
        """
        Returns the atmospheric density at a given altitude using the U.S. Standard Atmosphere (Mars) model.
        """
        # Replace with your preferred atmospheric model or interpolation method
        # Here's an example using the U.S. Standard Atmosphere (Mars) model:
        h = z + self.R_mars  # Convert altitude to geopotential altitude
        if h < 0:
            return 0.0  # No atmosphere below ground
        elif h <= 25000:
            # Use exponential profile for lower altitudesw
            return 0.01225 * np.exp(-h / 6500)
        else:
            # Use constant density for higher altitudes
            return 0.001

    def C_l(self, mach, alpha):
        """
        Returns the lift coefficient based on Mach number and angle of attack.
        """
        # Replace with your chosen lift coefficient model based on Mach number and spacecraft geometry
        # Here's an example using a simplified linear model:
        return 0.8 * alpha - 0.2  # Adjust coefficients based on your model

    def C_d(self, mach):
        """
        Returns the drag coefficient based on Mach number.
        """
        # Replace with your chosen drag coefficient model based on Mach number and spacecraft geometry
        # Here's an example using a basic model:
        return 0.8  # Adjust coefficient based on your model
    def T(self, z):
        """
        Returns the atmospheric temperature at a given altitude using the U.S. Standard Atmosphere (Mars) model.
        """
        h = z + self.R_mars  # Convert altitude to geopotential altitude
        if h < 0:
            return 140  # Constant temperature below ground
        elif h <= 25000:
            # Use linear temperature profile for lower altitudes
            return 140 - 0.0028 * h / 1000  # Adjust coefficients based on actual model
        else:
            # Use constant temperature for higher altitudes
            return 110  # Adjust constant value according to actual model
    def A_ref(self):
        # ... (set a constant value or use a calculation based on your scenario) ...
        return 15.9  # Replace with the appropriate value

    def update_state(current_state, acceleration, time_step):
        """
        This function updates the spacecraft state based on acceleration and time step.

        Args:
            current_state: A numpy array representing the current state (x, y, z, vx, vy, vz).
            acceleration: A numpy array representing the acceleration (ax, ay, az).
            time_step: The time step between calculations (seconds).

        Returns:
            A numpy array representing the updated state (x, y, z, vx, vy, vz) at the next time step.
        """

        
        # Update velocity
        new_velocity = current_state[3:6] + acceleration * time_step

        # Update position
        new_position = current_state[:3] + new_velocity*time_step
        


        # Combine updated state
        new_state = np.concatenate((new_position, new_velocity))
        # new_state = np.append(new_position, new_velocity)

        return new_state

    def dynamics_model(self, state, bank_angle):
        """
        Updates the state of the spacecraft based on the dynamics model.
        """
        
        x, y, z, vx, vy, vz = state 
        

        # Compute forces
        v= np.sqrt(vx**2+vy**2+vz**2)
        gamma = np.arctan2(state[5], state[3])
        gravity = self.G * self.mass / (z + self.R_mars)**2
        a = np.sqrt(1.3 * 8.314 * self.T(z)) # Calculate sound speed 
        mach = v / a  # Calculate Mach number based on temperature
        alpha = bank_angle  # Assume angle of attack equals bank angle
        C_d = self.C_d(mach)  # Use calculated drag coefficient
        drag = -0.5 * C_d * self.A_ref() * self.rho(z) * v**2 
        lift = 0.5 * self.C_l(mach, alpha) * self.A_ref ()* self.rho(z) * v**2 # * np.cos(bank_angle)

        acc_x = drag * vx / v
        acc_y = drag * vy / v + lift * np.sin(bank_angle)
        acc_z = -gravity + drag * vz / v + lift * np.cos(bank_angle)
        acc = np.array([acc_x, acc_y, acc_z])
        step=self.dt
        new_state= MarsEntry.update_state(state,acc,step)

        return new_state


testt= MarsEntry(mass=2200)
next_state=testt.dynamics_model([0.0, 0.0, 125e3, 4000.0, -1000.0, -250.0],np.deg2rad(50))
print(next_state)