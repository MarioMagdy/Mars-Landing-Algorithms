import numpy as np


def handle_thrust_controls():

    global max_thrust
    global left_wing_up_thruster_power, left_wing_down_thruster_power
    global right_wing_up_thruster_power, right_wing_down_thruster_power
    global top_left_thruster_power, top_right_thruster_power
    global bottom_left_thruster_power, bottom_right_thruster_power
    global Thruster
    global y_position, x_velocity, y_velocity
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



