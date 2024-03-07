import numpy as np

class MarsEntryGuidance:
    """
    This class represents a simplified Mars entry guidance system inspired by Apollo and MSL approaches.
    """

    def __init__(self, mass, dt=0.1):
        self.mass = mass
        self.dt = dt

        # Constants
        self.G = 6.6743e-11  # Gravitational constant
        self.R_mars = 3389e3  # Mars radius
        self.max_bank_angle = np.pi / 6

        # Target point and tolerance
        self.target_x = 0
        self.target_y = 0
        self.target_z = -10
        self.tolerance = 0.1

        # Gain schedule (example)
        self.K_downrange = 0.1
        self.K_heading = 0.05
        self.K_roll

        # Initial state (placeholder)
        self.state = None

    def set_initial_state(self, x, y, z, vx, vy, vz, gamma):
        """
        Sets the initial state of the spacecraft.
        """
        self.state = np.array([x, y, z, vx, vy, vz, gamma])

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
            # Use exponential profile for lower altitudes
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

    def dynamics_model(self, state, bank_angle):
        """
        Updates the state of the spacecraft based on the dynamics model.
        """
        x, y, z, vx, vy, vz, gamma = state

        # Compute forces
        v= np.sqrt(vx**2+vy**2+vz**2)
        gravity = self.G * self.mass / (z + self.R_mars)**2
        mach = v / np.sqrt(gamma * 287 * self.T(z))  # Calculate Mach number based on temperature
        alpha = bank_angle  # Assume angle of attack equals bank angle
        C_d = self.C_d(mach)  # Use calculated drag coefficient
        drag = -0.5 * C_d * self.A_ref() * self.rho(z) * v**2 / v
        lift = 0.5 * self.C_l(mach, alpha) * self.A_ref ()* self.rho(z) * v**2 * np.cos(bank_angle)

        acc_x = drag * vx / v
        acc_y = drag * vy / v + lift * np.sin(bank_angle)
        acc_z = -gravity + drag * vz / v + lift * np.cos(bank_angle)
        acc = np.array([acc_x, acc_y, acc_z])
        # Compute new state using Runge-Kutta method
        k1 = acc * self.dt
        k2 = self.dynamics_model(state + k1 / 2, bank_angle)[3:] * self.dt
        k3 = self.dynamics_model(state + k2 / 2, bank_angle)[3:] * self.dt
        k4 = self.dynamics_model(state + k3, bank_angle)[3:] * self.dt
        new_state = state + (k1 + 2 * k2 + 2 * k3 + k4) / 6
        return new_state

    def predict_state(self, state, bank_angle):
        """
        Predicts the state of the spacecraft at the target point using the current bank angle.
        """
        predicted_state = state
        while predicted_state[2] < self.target_z:
            predicted_state = self.dynamics_model(predicted_state, bank_angle)
        return predicted_state

    def bank_angle_control(self, state, predicted_state):
        """
        Calculates and adjusts the bank angle based on the predicted state and error correction.
        """
        # Downrange error
        error_downrange = predicted_state[0] - self.target_x

       

        # Adaptive gain based on flight phase (example)
        if state[2] > -50e3:
            self.K_downrange = 0.2
        else:
            self.K_downrange = 0.1

        # Update bank angle with downrange and heading control
        bank_angle = bank_angle - self.K_downrange * error_downrange 

        # Limit bank angle
        bank_angle = np.clip(bank_angle, -self.max_bank_angle, self.max_bank_angle)

        return bank_angle
    def bank_angle_control2(self, state, predicted_state):
            """
            Calculates and adjusts the bank angle based on the predicted state and error correction.
            """
            # Downrange error
            # error_downrange = predicted_state[0] - self.target_x

           

            # Range-to-go
            R = np.sqrt(state[0]**2 + state[1]**2 + state[2]**2) - self.R_mars


            # Flight path angle
            gamma = np.arctan2(state[5], state[3])
            # Lift-to-drag ratio
            L_D = 0.24

            # Predicted range-to-go
            R_p = predicted_state[0]

            # Desired vertical component of lift-to-drag ratio
            L_D_v = L_D*np.sin(gamma)+self.K_downrange * (R_p - R) / L_D 



            # Bank angle from vertical component of lift-to-drag ratio
            bank_angle = np.arccos((L_D_v) / L_D) * self.K_roll

            # Limit bank angle
            bank_angle = np.clip(bank_angle, -self.max_bank_angle, self.max_bank_angle)

            return bank_angle
    def guide(self):
            """
            Executes the guidance loop to steer the spacecraft towards the target point.
            """
            if self.state is None:
                raise ValueError("Initial state not set!")

            bank_angle = 0

            while self.state[2] < self.target_z:
               

                # Predict state at target
                predicted_state = self.predict_state(self.state, bank_angle)

                # Calculate and adjust bank angle
                bank_angle = self.bank_angle_control(self.state, predicted_state)
                print(f'{bank_angle}')
                # Apply bank angle and update state
                self.state = self.dynamics_model(self.state, bank_angle)

            # Check landing success
            if abs(self.state[0] - self.target_x) < self.tolerance and \
            abs(self.state[1] - self.target_y) < self.tolerance and \
            abs(self.state[2] - self.target_z) < self.tolerance:
                print("Successful landing!")
            else:
                print("Landing accuracy outside tolerance.")


def test_guidance(target_x, target_y, target_z):
    """
    Tests the MarsEntryGuidance class with a given target point.
    """
    # Set mass and initial state (adjust values as needed)
    mass = 2200  # kg
    # initial_state = np.array([0, 0, -100000, 5000, -1.22])  # Initial position, velocity, and flight path angle

    # Create guidance object and set target
    guidance = MarsEntryGuidance(mass)
    # guidance.set_initial_state= lambda x, y, z, v, gamma : initial_state
    guidance.set_initial_state(0, 0, -100000, 5000, -1.22)
    guidance.target_x = target_x
    guidance.target_y = target_y
    guidance.target_z = target_z
    
    # Set atmospheric and reference area (replace with realistic values)
    guidance.rho = lambda z: 0.001  # Placeholder density
    guidance.C_l = lambda mach, alpha: 0.5  # Placeholder lift coefficient
    
    guidance.guide()


# Choose a realistic target point (e.g., Jezero Crater or Elysium Planitia)
target_x = 181.345  # degrees East longitude
target_y = 18.995  # degrees North latitude
target_z = -4300  # meters (relative to Martian surface datum)

# Run the test
test_guidance(target_x, target_y, target_z)
