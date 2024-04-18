import numpy as np

class Thruster:
    def __init__(self, power):
        self.power = power

def handle_thrust_controls(altitude, vertical_velocity, horizontal_velocity, spacecraft_orientation, dt):

    global max_thrust
    global left_wing_up_thruster_power, left_wing_down_thruster_power
    global right_wing_up_thruster_power, right_wing_down_thruster_power
    global top_left_thruster_power, top_right_thruster_power
    global bottom_left_thruster_power, bottom_right_thruster_power
    global Thruster
    global y_position, x_velocity, y_velocity
    global desired_altitude, desired_descent_rate
    global spacecraft_orientation
    
    # Extract spacecraft state information
    altitude = y_position
    vertical_velocity = y_velocity
    horizontal_velocity = x_velocity
    orientation = spacecraft_orientation
    
    # Reset thruster powers to zero
    left_wing_up_thruster_power = 0.0
    left_wing_down_thruster_power = 0.0
    right_wing_up_thruster_power = 0.0
    right_wing_down_thruster_power = 0.0
    top_left_thruster_power = 0.0
    top_right_thruster_power = 0.0
    bottom_left_thruster_power = 0.0
    bottom_right_thruster_power = 0.0


    thrusters = [
            np.array([0, 1, 0]),  # left_wing_up_thruster_power
            np.array([0, -1, 0]),  # left_wing_down_thruster_power
            np.array([0, 1, 0]),  # right_wing_up_thruster_power
            np.array([0, -1, 0]),  # right_wing_down_thruster_power
            np.array([-1, 0, 0]),  # top_left_thruster_power
            np.array([1, 0, 0]),  # top_right_thruster_power
            np.array([-1, 0, 0]),  # bottom_left_thruster_power
            np.array([1, 0, 0])   # bottom_right_thruster_power
        ]
        
    # Adjust these key codes based on your desired controls
    # Wing thrusters control vertical movement and orientation
    # Simple control logic based on altitude, velocity, and orientation
    if altitude > 10000:  # Higher altitude
        # Maintain stability
         top_left_thruster_power = 50.0
         top_right_thruster_power = 50.0
         bottom_left_thruster_power = 50.0
         bottom_right_thruster_power = 50.0
    else:
         # Lower altitude
         if vertical_velocity < -10:  # Descending too fast
             # Apply additional thrust to slow down descent
             left_wing_up_thruster_power = 80.0
             right_wing_up_thruster_power = 80.0
         elif vertical_velocity > 10:  # Ascending too fast
             # Apply additional thrust to slow down ascent
             left_wing_down_thruster_power = 80.0
             right_wing_down_thruster_power = 80.0
         else:
             # Maintain vertical stability
             left_wing_up_thruster_power = 50.0
             right_wing_up_thruster_power = 50.0
             left_wing_down_thruster_power = 50.0
             right_wing_down_thruster_power = 50.0


    # Top and bottom thrusters control horizontal movement and orientation
    active_thrusters = []
    if spacecraft_orientation < 0:  # Roll left
        active_thrusters.extend([0, 1, 4, 7])  # Left thrusters
    elif spacecraft_orientation > 0:  # Roll right
        active_thrusters.extend([2, 3, 5, 6])  # Right thrusters
        
        # Apply thrust commands to active thrusters for orientation
        for index in active_thrusters:
            thrusters[index].power = 100.0  # Full power



    # PD controller gains
    kp_altitude = 0.1  # Proportional gain for altitude control
    kd_altitude = 0.05  # Derivative gain for altitude control
    
    # Calculate altitude error and velocity error
    altitude_error = desired_altitude - altitude
    velocity_error = desired_descent_rate - vertical_velocity
    
    # PD control law for altitude and velocity
    altitude_control_input = kp_altitude * altitude_error + kd_altitude * velocity_error
    
    # Thruster commands based on altitude control input
    thrusters = [0.0] * 8  # Initialize thruster commands
    thrusters[0] = altitude_control_input  # Adjust left_wing_up_thruster_power
    thrusters[2] = altitude_control_input  # Adjust right_wing_up_thruster_power
    
    thrusters = optimize_thruster_commands(thrusters)
    
    return thrusters

def optimize_thruster_commands(initial_commands):
    max_iterations = 100  # Maximum number of iterations
    learning_rate = 0.01  # Learning rate for gradient descent
    
    thruster_commands = initial_commands
    
    for i in range(max_iterations):
        # Compute gradients of the cost function with respect to thruster commands
        gradients = compute_gradients(thruster_commands)
        
        # Update thruster commands using gradient descent
        thruster_commands -= learning_rate * gradients
        
        # Enforce constraints ( thruster power limits)
        thruster_commands = enforce_constraints(thruster_commands)
        
    return thruster_commands

def compute_gradients(thruster_commands):
    #the cost function represents the deviation of altitude from the desired altitude
    
    # Perturb each thruster command slightly
    perturbation = 0.01
    gradients = np.zeros(8)
    for i in range(8):
        # Perturb thruster command i
        perturbed_commands = np.copy(thruster_commands)
        perturbed_commands[i] += perturbation
        
        # Calculate cost function for perturbed commands
        cost_perturbed = cost_function(perturbed_commands)
        
        # Calculate cost function for original commands
        cost_original = cost_function(thruster_commands)
        
        # Approximate gradient as change in cost function divided by perturbation
        gradients[i] = (cost_perturbed - cost_original) / perturbation
    
    return gradients

def cost_function(thruster_commands):
    #cost function representing the deviation of altitude from the desired altitude

    global altitude , desired_altitude
    
    # Calculate cost as the absolute difference between current altitude and desired altitude
    return abs(altitude - desired_altitude)  


def enforce_constraints(thruster_commands):
    # Placeholder function for enforcing constraints 
    return np.clip(thruster_commands, 0, 100)


