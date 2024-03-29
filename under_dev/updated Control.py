import time
import numpy as np
import math


# Constants
mass = 1000  # kg
g_mars = 3.71  # m/s^2 (Martian gravity)
drag_coeff = 0.5
density = 0.02  # kg/m³
HEAT_SHIELD_MASS = 500  # kg, mass of the heat shield
dt = 0.01  # (seconds) time step
t_max = 300  # (seconds) total simulation time
density = 0.02 #kg/m³

# Spacecraft properties for Mars mission
mass_mars = 3000  # kg, mass of the spacecraft for Mars mission
area_mars = 15.0  # m^2, area of the spacecraft's aerodynamic surfaces for Mars mission
lift_coefficient_mars = 1.2  # Assumed constant lift coefficient for Mars mission(it should be calculated by guidance)
drag_coefficient_mars = 1.0 #  Assumed constant drag coefficient for Mars mission(it should be calculated by guidance)


class EntryGuidanceSystem:
    def __init__(self):
        
        pass

    def calculate_bank_angle(self):
        # Simulated calculation of the required bank angle
        return input()  # degrees


class MarsLanderControl:
    
    def __init__(self):
        self.mass_spacecraft = 1000  # Mass of the spacecraft (kg)
        self.delta_v = 7800  # Desired change in velocity (m/s)
        self.thrust_force_per_thruster = 0.0  # Initialize thrust force per thruster
        self.thrust_force = 0.0  # Example a constant thrust force (in newtons)
        self.rcs_thrusters = {'left_front': False, 'left_rear': False,
                              'right_front': False, 'right_rear': False}

        self.bank_angle = 0.0  # Current bank angle in degrees
        self.setpoint = 0.0  # Desired bank angle (commanded value)
        
        self.kp = 0.1  # Proportional gain
        self.ki = 0.01  # Integral gain
        self.kd = 0.05  # Derivative gain
        self.integral_term = 0.0  # Integral term accumulator
        
    # Function to approximate atmospheric density based on altitude
    def get_atmospheric_density_mars(self, altitude):
        # Simple exponential model (not real-time data)
        scale_height = 11.1  # km, scale height for Mars' atmosphere
        return density * np.exp(-altitude / (scale_height * 1000))
        
    
    def calculate_bank_angle(self):
        # Placeholder function to calculate the commanded bank angle
        # input from guidance team
        return input()  # degrees
    
    
    def clamp(self, val, upper, lower):
        return max(min(val, upper), lower)
    
    
    def update_pid(self, last_error, st, last_I, last_D, outputlim):

        self.outputlim = outputlim
        # Compute error
        error = self.bank_angle - self.setpoint
        # Compute proportional term
        P = self.kp * error
        # Compute integral term
        I = last_I + self.ki * ((error + last_error) / 2) * self.st
        # Compute derivative term
        D = (2 * self.kd * (error - last_error) + (2 * self.t_max - self.st) * last_D) / (2 * self.t_max + self.st)
        # Compute output signal
        pid_output = P + I + D
        pid_output = self.clamp(pid_output, self.outputlim, -self.outputlim)
        return pid_output, error, I, D
    
    
    def execute_bank_angle(self, commanded_bank_angle):
        # Placeholder function to execute the commanded bank angle
        # Here, we adjust the spacecraft's orientation using RCS 
        self.setpoint = commanded_bank_angle
        pid_output = self.update_pid()
        self.update_rcs_thrusters()
        print(f"Executing bank angle: {self.bank_angle:.2f} degrees")
        
    def calculate_thrust_forces(self):
        # Calculate thrust forces for each active thruster
        active_thrusters = [thruster for thruster, firing in self.rcs_thrusters.items() if firing]
        total_thrust = len(active_thrusters) * self.thrust_force
        acceleration_due_to_thrusters = total_thrust / mass
        self.acceleration += acceleration_due_to_thrusters
        print(f"Total thrust force: {total_thrust:.2f} N")
        
    def calculate_required_thrust(self):
        # Calculate the total required thrust based on desired delta_v
        #The direction of thrust is perpendicular to the current velocity vector for a pure orbit change.
        gravitational_force = self.mass_spacecraft * g_mars  # Earth's gravity
        self.thrust_force_per_thruster = (self.delta_v - gravitational_force) / 4
    
    def fire_thrusters(self):
        # Determine which RCS thrusters to fire based on the commanded bank angle
        if self.bank_angle > 0:
            self.rcs_thrusters['left_front'] = True
            self.rcs_thrusters['right_rear'] = True
        elif self.bank_angle < 0:
            self.rcs_thrusters['right_front'] = True
            self.rcs_thrusters['left_rear'] = True
        else:
            # No bank angle, no thrusters firing
            for thruster in self.rcs_thrusters:
                self.rcs_thrusters[thruster] = False
                
    def update_orientation(self, bank_angle):
        # Update orientation based on the bank angle
        # Here we assume a simple model: directly updating the orientation quaternion
        delta_quaternion = np.array([np.cos(bank_angle / 2), 0, 0, np.sin(bank_angle / 2)])
        self.orientation = np.quaternion(*delta_quaternion) * np.quaternion(*self.orientation)

    # Function to calculate lift & Drag force on Mars
    def calculate_aerodynamic_forces_mars(self, velocity, angle_of_attack, altitude):
        atmospheric_density = self.get_atmospheric_density_mars()
        dynamic_pressure_mars = 0.5 * atmospheric_density * velocity**2
        lift_force_mars = dynamic_pressure_mars * area_mars * lift_coefficient_mars * np.sin(angle_of_attack)
        drag_force_mars = dynamic_pressure_mars * area_mars * lift_coefficient_mars * np.cos(angle_of_attack)
        return lift_force_mars, drag_force_mars


    # Function to adjust lift vector and descent angle for Mars entry
    def mars_entry_control(self, velocity, position, angle_of_attack):
        # Adjust angle of attack based on altitude specific to Mars entry profile
        if position > 80000:  # High altitude on Mars, low angle for shallow entry
            angle_of_attack = np.radians(5)
        elif position > 40000:  # Mid altitude on Mars, increase angle for more lift
            angle_of_attack = np.radians(15)
        else:  # Low altitude on Mars, decrease angle to prepare for landing
            angle_of_attack = np.radians(10)
        
        # Calculate lift force on Mars and adjust trajectory
        lift_force_mars = self.calculate_lift_force_mars(self, velocity, angle_of_attack)
        vertical_velocity_change_mars = (lift_force_mars / mass_mars) * np.cos(angle_of_attack)
        horizontal_velocity_change_mars = (lift_force_mars / mass_mars) * np.sin(angle_of_attack)
    
        return vertical_velocity_change_mars, horizontal_velocity_change_mars, angle_of_attack

    
    # Function to simulate heat shield jettison
    def jettison_heat_shield(self, mass_with_shield):
        mass_without_shield = mass_with_shield - HEAT_SHIELD_MASS
        print(f"Heat shield jettisoned. New spacecraft mass: {mass_without_shield} kg")
        return mass_without_shield
    
    def main(self):
        """
        Simulates the lander's descent in a loop.
        """

        # Control logic: determine desired orientation
        desired_orientation = np.array([1, 0, 0, 0])  # Quaternion for upright

        # Calculate bank angle
        commanded_bank_angle = self.calculate_bank_angle()
        self.execute_bank_angle(commanded_bank_angle)
            
        self.calculate_thrust_forces()
        self.calculate_required_thrust()
        print(f"Required Thrust per Thruster: {self.thrust_force_per_thruster:.2f} N")
        self.fire_thrusters()
        
        time.sleep(2)  # Simulating descent

        # Update estimated orientation
        self.update_orientation(self.bank_angle)

        # Update state
        self.update_pid()
        

if __name__ == "__main__":
    MarsLander = MarsLanderControl()
    MarsLander.prev_error = 0.0  # Initialize previous error for derivative term
    MarsLander.main()
    
    # Simulate Mars entry phase with control adjustments and variable atmospheric density
    entry_velocity_mars = 5500  # m/s, initial entry velocity for Mars mission
    jettison_altitude = 70000  # m, altitude at which heat shield is jettisoned
    
    # Initialize variables for Mars simulation
    velocity_mars = entry_velocity_mars
    position_mars = 120000  # Starting at high altitude specific to Mars entry profile
    
    # Simulation loop for Mars entry with aerodynamic forces calculation
    time_step_mars = dt  # s, simulation time step
    total_time_mars = t_max  # s, total simulation time for Mars entry
    angle_of_attack_mars = np.radians(5)  # Initial angle of attack

    for t in range(0, total_time_mars, time_step_mars):
        lift_force_mars, drag_force_mars = MarsLander.calculate_aerodynamic_forces_mars(velocity_mars, angle_of_attack_mars, position_mars)
    
    # Update velocity and position based on gravity and aerodynamic forces (simplified model)
    velocity_change_due_to_gravity = g_mars * time_step_mars
    velocity_change_due_to_drag = drag_force_mars / mass_mars * time_step_mars
    
    velocity_mars -= (velocity_change_due_to_gravity + velocity_change_due_to_drag)







