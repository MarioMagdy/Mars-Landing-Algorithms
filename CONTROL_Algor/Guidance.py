import numpy as np

class MarsEntryGuidance:
    """
    This class represents a simplified Mars entry guidance system inspired by Apollo and MSL approaches.
    """

    def __init__(self, mass, dt=0.01):
        self.mass = mass
        self.dt = dt

        # Constants
        self.G = 6.6743e-11  # Gravitational constant
        self.R_mars = 3389e3  # Mars radius
        self.max_bank_angle = np.pi / 6
        self.bank_reversal_interval = 10  # Time interval for bank reversals in seconds
        self.last_reversal_time = 0

        # Constants for Mars' atmosphere
        self.MARS_LAPSE_RATE = -6.5 / 1000 # Temperature decrease in °C per meter

        # Base temperature at Mars' surface level (example value)
        self.base_temperature = -63 # in °C, approximate average surface temperature
        # Target point and tolerance
        self.target_x = 0
        self.target_y = 0
        self.target_z = 10000
        self.tolerance = 0.1

        # Gain schedule (example)
        self.K_downrange = 0.1
        self.K_heading = 0.05
        self.K_roll =1

        # Initial state
        self.state = np.zeros(12)  # state vector: [position, velocity, angles, angular rates, AoA, sideslip, bank angle]
        self.current_bank_angle = 0

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
    def mach_num(self,v,altitude):
        # Calculate the temperature at the given altitude
        temperature = self.base_temperature + (self.MARS_LAPSE_RATE * altitude)
        # Calculate the speed of sound using the temperature
        speed_of_sound = 331.3 * np.sqrt(1 + (np.abs(temperature) / 273.15))
        mach_num=v/speed_of_sound
        return mach_num 
    def update_state(self,current_state, acceleration, time_step):
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
        new_velocity = acceleration * time_step

        # Update position
        new_position =  new_velocity*time_step
        
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
        mach = self.mach_num(v=v,altitude=z)  # Calculate Mach number based on temperature
        alpha = np.radians(bank_angle)  # Assume angle of attack equals bank angle
        C_d = self.C_d(mach)  # Use calculated drag coefficient
        drag = 0.5 * C_d * self.A_ref() * self.rho(z) * v**2 
        lift = 0.5 * self.C_l(mach, alpha) * self.A_ref ()* self.rho(z) * v**2# * np.cos(bank_angle)

        acc_x = drag * vx / v
        acc_y = drag * vy / v + lift * np.sin(bank_angle)
        acc_z = -gravity + drag * vz / v + lift * np.cos(bank_angle)
        acc = np.array([acc_x, acc_y, acc_z])
        # another way to compute new state is by integration for the acc using the update state function:
        new_state=self.update_state(state,acc,self.dt)
        L_D=lift/drag

        return [new_state,L_D]

    def predict_state(self, state, bank_angle):
        """
        Predicts the state of the spacecraft at the target point using the current bank angle.
        """
        predicted_state = state
        while predicted_state[2] > self.target_z:
            predicted_state = self.dynamics_model(predicted_state, bank_angle)[0]
        return predicted_state

    def bank_angle_control2(self, state, predicted_state,L_D):
            """
            Calculates and adjusts the bank angle based on the predicted state and error correction.
            """
           
            # Target Range-to-go (the downrange distance remaining between the spacecraft's current position and the target landing point.)
            R = np.sqrt((self.target_x-state[0])**2 + (self.target_y-state[2])**2)
            x, y, z, vx, vy, vz = state


            v= np.sqrt(vx**2+vy**2+vz**2)

            # Flight path angle
            gamma = np.arctan2(state[5], state[3])
            # Lift-to-drag ratio
            L_D_R = 0.24

            # Predicted range-to-go
            R_p = np.sqrt(( predicted_state[0]-state[0])**2 + ( predicted_state[2]-state[2])**2)

            # Desired vertical component of lift-to-drag ratio
            L_D_v = L_D_R*np.sin(gamma)+self.K_downrange * (R-R_p) / v 

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

            while self.state[2] > self.target_z:
                print('working')
               
                print(f'{self.state}')
                # Predict state at target
                predicted_state = self.predict_state(self.state, bank_angle)
                print(f'{predicted_state}')
                L_D=self.dynamics_model(self.state, bank_angle)[1]
                print(f'L/D={L_D}')
                bank_angle = self.bank_angle_control2(self.state, predicted_state,L_D=L_D)
                bank_angle=np.rad2deg(bank_angle)
                print(f'{bank_angle}')
                self.state = self.dynamics_model(self.state, bank_angle)[0]
                print(f'{self.state}')
                return bank_angle
    
    def guidance(self, current_state, current_time):
        
        # Extract position and velocity
        pos = current_state[:3]
        vel = current_state[3:6]

        # Calculate downrange error
        downrange_error = np.sqrt((self.target_x - pos[0]) ** 2 + (self.target_y - pos[1]) ** 2)
        
        # Calculate heading error (simplified)
        target_heading = np.arctan2(self.target_y - pos[1], self.target_x - pos[0])
        current_heading = np.arctan2(vel[1], vel[0])
        heading_error = target_heading - current_heading

        # Bank angle control (example proportional control)
        commanded_bank_angle = self.K_downrange * downrange_error + self.K_heading * heading_error
        commanded_bank_angle = np.clip(commanded_bank_angle, -self.max_bank_angle, self.max_bank_angle)

        # Bank reversal logic
        if current_time - self.last_reversal_time >= self.bank_reversal_interval:
            commanded_bank_angle = -self.current_bank_angle  # Reverse bank angle
            self.last_reversal_time = current_time  # Update reversal time
        
        # Roll control ( based on heading )
        roll_error = commanded_bank_angle - self.current_bank_angle
        roll_cmd = self.K_roll * roll_error
        
        # Update state with new bank angle
        self.current_bank_angle = commanded_bank_angle
        current_state[10] = self.current_bank_angle

        return current_state
    
  
    def run_guidance(self, initial_state, duration, dt, output_file):
        time = 0
        state = initial_state

        with open(output_file, 'w') as f:
            f.write("Time,PositionX,PositionY,PositionZ,VelocityX,VelocityY,VelocityZ,OmegaX,OmegaY,OmegaZ,AoA,Sideslip,BankAngle\n")
            while time < duration:
                state = self.guidance(state, time)
                f.write(f"{time},{','.join(map(str, state))}\n")
                time += dt

# Instantiate the guidance system
guidance_system = MarsEntryGuidance(mass=800)

#initial state: position (m), velocity (m/s), angles (rad), angular rates (rad/s), AoA (rad), sideslip (rad), bank angle (rad)
initial_state = np.array([120000, 0, 0, 100, 0, -5500, 0, 0, 0, 0, 0, 0])

# Run guidance and store the data in a file
output_file = 'guidance_output.csv'
guidance_system.run_guidance(initial_state, duration=100, dt=0.01, output_file=output_file)

print(f"Guidance data stored in {output_file}")

