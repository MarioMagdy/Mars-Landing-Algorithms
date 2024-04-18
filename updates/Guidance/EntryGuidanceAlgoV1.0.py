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
        self.target_z = 10000
        self.tolerance = 0.1

        # Gain schedule (example)
        self.K_downrange = 0.1
        self.K_heading = 0.05
        self.K_roll =1

        # Initial state (placeholder)
        self.state = None

    def set_initial_state(self, x, y, z, vx, vy, vz):
        """
        Sets the initial state of the spacecraft.
        """
        self.state = np.array([x, y, z, vx, vy, vz])

    def reference_bank_angle(self,  state, ):
        """Bank angle function for open loop guidance that simply returns
           the reference bank angle"""
        x, y, z, vx, vy, vz = state
        v= np.sqrt(vx**2+vy**2+vz**2)
        if v >= 3500:
            return np.deg2rad(75)
        elif v <= 1500:
            return np.deg2rad(50)
        else:
            return np.deg2rad(50 + (75-50)*(v-1500)/(3500-1500))
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

        # Update position
        new_position = current_state[:3] + current_state[3:6] * time_step + 0.5 * acceleration[:3] * time_step**2

        # Update velocity
        new_velocity = current_state[3:6] + acceleration * time_step

        # Combine updated state
        new_state = np.concatenate((new_position, new_velocity))

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
        mach = v / np.sqrt(gamma * 287 * self.T(z))  # Calculate Mach number based on temperature
        alpha = bank_angle  # Assume angle of attack equals bank angle
        C_d = self.C_d(mach)  # Use calculated drag coefficient
        drag = -0.5 * C_d * self.A_ref() * self.rho(z) * v**2 / v
        lift = 0.5 * self.C_l(mach, alpha) * self.A_ref ()* self.rho(z) * v**2 * np.cos(bank_angle)

        acc_x = drag * vx / v
        acc_y = drag * vy / v + lift * np.sin(bank_angle)
        acc_z = -gravity + drag * vz / v + lift * np.cos(bank_angle)
        acc = np.array([acc_x, acc_y, acc_z])
        # there are two methods to compute the new state make sure to use just one of them and comment the other
        # Compute new state using Runge-Kutta method:
        # k1 = acc * self.dt
        # k2 = self.dynamics_model(state + k1 / 2, bank_angle)[3:] * self.dt
        # k3 = self.dynamics_model(state + k2 / 2, bank_angle)[3:] * self.dt
        # k4 = self.dynamics_model(state + k3, bank_angle)[3:] * self.dt
        # new_state = state + (k1 + 2 * k2 + 2 * k3 + k4) / 6
        # another way to compute new state is by integration for the acc using the update state function:
        new_state=self.update_state(state,acc,self.dt)

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
           
            # Target Range-to-go (the downrange distance remaining between the spacecraft's current position and the target landing point.)
            R = np.sqrt((self.target_x-state[0])**2 + (self.target_y-state[1])**2)


            # Flight path angle
            gamma = np.arctan2(state[5], state[3])
            # Lift-to-drag ratio
            L_D = 0.24

            # Predicted range-to-go
            R_p = np.sqrt(( predicted_state[0]-state[0])**2 + ( predicted_state[1]-state[1])**2)

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

            bank_angle = self.reference_bank_angle(self.state)

            while self.state[2] < self.target_z:
               

                # Predict state at target
                predicted_state = self.predict_state(self.state, bank_angle)
                bank_angle = self.bank_angle_control2(self.state, predicted_state)
                print(f'{bank_angle}')
                self.state = self.dynamics_model(self.state, bank_angle)
                # if abs(self.predicted_state[0] - self.target_x) < self.tolerance and \
                # abs(self.predicted_state[2] - self.target_z) < self.tolerance:
                #     print(f'{bank_angle , self.state[2] }')
                    
                # else:
                    
                #     # Calculate and adjust bank angle
                #     bank_angle = self.bank_angle_control2(self.state, predicted_state)
                #     print(f'{bank_angle}')
                #     # Apply bank angle and update state
                #     self.state = self.dynamics_model(self.state, bank_angle)
                    

            # Check landing success
            if abs(self.state[0] - self.target_x) < self.tolerance and \
            abs(self.state[1] - self.target_y) < self.tolerance and \
            abs(self.state[2] - self.target_z) < self.tolerance:
                print("Successful landing!")
                print(f'{bank_angle}')
            else:
                print("Landing accuracy outside tolerance.")
                print(f'{bank_angle}')


MNG= MarsEntryGuidance(mass=2200)
MNG.set_initial_state(0.0, 0.0, 125e3, 4000.0, -1000.0, -250.0)
MNG.guide()
# def test_guidance_algorithm(mars_guidance, initial_state, target_state, tolerance, max_iterations=1000):
#   """
#   This function tests the Mars entry guidance algorithm by simulating the descent process.

#   Args:
#       mars_guidance: An instance of your MarsEntryGuidance class.
#       initial_state: A numpy array representing the initial state of the spacecraft (x, y, z, vx, vy, vz).
#       target_state: A numpy array representing the target landing point (x, y, z).
#       tolerance: The maximum allowed error in each dimension for landing success (x, y, z).
#       max_iterations: The maximum number of iterations allowed for the simulation.

#   Returns:
#       A dictionary containing:
#           success: True if the spacecraft landed within tolerance, False otherwise.
#           iterations: The number of iterations it took to reach the target or reach the maximum.
#           final_state: The state of the spacecraft at the end of the simulation.
#   """
#   # Set initial state
#   mars_guidance.set_initial_state(*initial_state)

#   # Simulation loop
#   for iteration in range(max_iterations):
#     # Predict state at target point
#     predicted_state = mars_guidance.predict_state(mars_guidance.state, mars_guidance.reference_bank_angle(mars_guidance.state))

#     # Check landing success
#     landing_error = np.abs(predicted_state[:3] - target_state[:3])
#     if all(error <= tolerance for error in landing_error):
#       return {
#           "success": True,
#           "iterations": iteration + 1,
#           "final_state": mars_guidance.state
#       }

#     # Calculate and adjust bank angle
#     mars_guidance.bank_angle_control(mars_guidance.state, predicted_state)

#     # Update state
#     mars_guidance.state = mars_guidance.dynamics_model(mars_guidance.state, mars_guidance.bank_angle_control(mars_guidance.state, predicted_state))

#   # Reached maximum iterations without landing
# #   return {
# #       "success": False,
# #       "iterations": max_iterations,
# #       "final_state": mars_guidance.state
# #   }


# # Define initial state (replace with your desired values)
# initial_state = np.array([0.0, 0.0, 125e3, 4000.0, -1000.0, -250.0])  # x, y, z, vx, vy, vz (meters)

# # Justification for initial state:
# #  - x, y: We assume a zero initial lateral position (0 meters) relative to the target.
# #  - z: Starting at -125 km above the Martian surface is a typical entry altitude for missions.
# #  - vx, vy: A hypersonic entry velocity of 4000 m/s (around Mach 12) is realistic for interplanetary travel.
# #  - vz: A negative vz (-250 m/s) indicates a downward velocity component.

# # Define target landing point (replace with your desired values)
# target_state = np.array([0.0, 0.0, -10.0])  # x, y, z (meters)

# # Tolerance for landing accuracy (replace with your desired values)
# tolerance = np.array([100.0, 100.0, 5.0])  # x, y, z (meters)

# # Justification for tolerance:
# #  - x, y: A landing accuracy of 100 meters in the horizontal plane is achievable, but challenging.
# #  - z: A vertical tolerance of 5 meters is a demanding target for pinpoint landing.

# # Maximum iterations for the simulation
# max_iterations = 1000

# # Create an instance of your MarsEntryGuidance class
# mars_guidance = MarsEntryGuidance(mass=2200.0)  # Replace mass with your spacecraft mass

# # Test the guidance algorithm using the test function
# # test_guidance_algorithm(mars_guidance, initial_state, target_state, tolerance, max_iterations)

# # # Print the test results
# # print(f"Landing Success: {test_result['success']}")
# # print(f"Iterations: {test_result['iterations']}")
# # print(f"Final State:\n {test_result['final_state']}")
