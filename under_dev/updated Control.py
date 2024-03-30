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
        # Placeholder function to calculate aerodynamic forces on Mars
        
        atmospheric_density = self.get_atmospheric_density_mars(altitude)
        dynamic_pressure_mars = 0.5 * atmospheric_density * velocity**2
        lift_force_mars = dynamic_pressure_mars * area_mars * lift_coefficient_mars * np.sin(angle_of_attack)
        drag_force_mars = dynamic_pressure_mars * area_mars * drag_coefficient_mars * np.cos(angle_of_attack)
        return lift_force_mars, drag_force_mars

    def mars_entry_control(self, velocity, position, angle_of_attack):
        # Placeholder function to control entry to Mars
        
        # Define default values for adjustments
        vertical_velocity_change_mars = 0.0
        horizontal_velocity_change_mars = 0.0

        # Adjust angle of attack based on altitude specific to Mars entry profile
        if position > 80000:  # High altitude on Mars, low angle for shallow entry
            angle_of_attack = np.radians(5)
        elif position > 40000:  # Mid altitude on Mars, increase angle for more lift
            angle_of_attack = np.radians(15)
        else:  # Low altitude on Mars, decrease angle to prepare for landing
            angle_of_attack = np.radians(10)
        
        # Calculate lift force on Mars and adjust trajectory
        lift_force_mars, _ = self.calculate_aerodynamic_forces_mars(velocity, angle_of_attack, position)
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
        self.fire_thrusters()
        
        time.sleep(2)  # Simulating descent

        # Update estimated orientation
        self.update_orientation(self.bank_angle)

        # Update state
        self.update_pid()
        
        # Simulation loop for Mars entry
        velocity_mars = 5500  # m/s, initial entry velocity for Mars mission
        position_mars = 120000  # Starting at high altitude specific to Mars entry profile
        angle_of_attack_mars = np.radians(5)  # Initial angle of attack
        
        # Simulate Mars entry phase with control adjustments and variable atmospheric density
        entry_velocity_mars = velocity_mars  # m/s, initial entry velocity for Mars mission
        jettison_altitude = 70000  # m, altitude at which heat shield is jettisoned
    
        
        for t in np.arange(0, t_max, dt):
            
            lift_force_mars, drag_force_mars = self.calculate_aerodynamic_forces_mars(velocity_mars, angle_of_attack_mars, position_mars)
            # Calculate aerodynamic forces and adjust trajectory
            vertical_vel_change, horizontal_vel_change, angle_of_attack_mars = self.mars_entry_control(velocity_mars, position_mars, angle_of_attack_mars)
            
            # Update velocity and position based on gravity and aerodynamic forces (simplified model)
            velocity_change_due_to_gravity = g_mars * dt # vertical velocity change
            velocity_change_due_to_drag = drag_force_mars / mass_mars * dt # Horizontal velocity change
            
            velocity_mars -= (velocity_change_due_to_gravity + velocity_change_due_to_drag)
            
            # Update velocity and position
            velocity_mars -= vertical_vel_change * dt
            position_mars -= horizontal_vel_change * dt
            
            # Update thruster magnitude based on current state (placeholder function)
            self.calculate_required_thrust()
            
            # Fire thrusters based on updated magnitude
            self.fire_thrusters()
            
            # Print current state or perform further actions as needed
            print(f"Time: {t:.2f}s, Altitude: {position_mars:.2f}m, Velocity: {velocity_mars:.2f}m/s, Angle of Attack: {np.degrees(angle_of_attack_mars):.2f}°")
            
            # Check for termination conditions (e.g., touchdown)
            if position_mars <= 0:
                print("Touchdown! Mars entry phase complete.")
                break

    








