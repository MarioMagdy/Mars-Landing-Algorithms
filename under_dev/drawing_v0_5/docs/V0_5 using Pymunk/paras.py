###TODO: 
# To make it more accurate we can make the gravity to the center of the planet not downwards so the orbiting effect can happen
slower= 1000
time_from_start = 0
fps = 60



# Define colors
WHITE = (255, 255, 255)
THRUSTER_EFFECT_COLOR = (255, 255, 0)  # Yellow color for visibility

# Constants
MARS_GRAVITY = 3.71  # m/s^2

## TODO: Should be affected by the oriantaion
AIR_RESISTANCE_COEFF = 3 # (adjustable for different air densities) 
LD_C = 10 # (adjustable)


SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 700
SCALE = 200  # meters per pixel (adjust for visual representation)


# User input (replace with prompts and error handling)
initial_altitude = 120000.0  # meters

# TODO: Get the accurate velocities
# total vel = 5333 = sqrt(initial_horizontal_velocity^2 + initial_vertical_velocity^2)
initial_horizontal_velocity = 100.0  # m/s (positive to the right)
initial_vertical_velocity = 533.0  # m/s (negative for downward)
max_thrust = 100.0  # m/s^2
spacecraft_mass = 1025  # kg
fuel_mass = 100.0  # kg # Hard to calculate real number so this is going to be arbtirary number 
fuel_consumption_rate = 1.0  # kg/s per unit of thrust (adjustable for engine efficiency)


control_ori_st= 0.08
control_pos_st= 0.001



# Simulation variables
x_position = SCALE*100  # meters
y_position = initial_altitude  # meters (matches initial altitude)
x_velocity = initial_horizontal_velocity
y_velocity = initial_vertical_velocity
fuel_remaining = fuel_mass # Hard to calculate real number so this is going to be arbtirary number 
total_fuel_time = 300 * slower  # Hard to calculate real number so this is going to be arbtirary number 
thrust_x = 0.0  # m/s^2 (horizontal thrust)
thrust_y = 0.0  # m/s^2 (vertical thrust)
orientation_change = 0.0 # change in orientation per dt
random_wind_factor = 2.0 # how much the wind affects the orientation of the spacecraft


# Colors    
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (133, 40, 20)  # For crash landing
YELLOW = (255, 255, 0)  # For successful landing
GREY = (170, 170, 170) # For
MARS_SURFACE_COLOR = (115, 55, 0)  # Rusty Mars surface color
ATMOSPHERE_COLOR = (255, 200, 180, 40)  # Semi-transparent white for atmosphere
THRUSTER_EFFECT_COLOR = (255, 255, 0)  # Yellow color for visibility


# New global variable for spacecraft orientation (in degrees)
spacecraft_orientation = 0.0








#### Controls Init
(left_wing_up_thruster_power, left_wing_down_thruster_power, right_wing_up_thruster_power,
 right_wing_down_thruster_power, top_left_thruster_power, top_right_thruster_power , 
 bottom_left_thruster_power, bottom_right_thruster_power) = (0,0,0,0,0,0,0,0)

# Assuming 'screen' is your Pygame display surface and 'spacecraft_position' is the current position of your spacecraft
thruster_powers = {
    'left_wing_up': left_wing_up_thruster_power,
    'left_wing_down': left_wing_down_thruster_power,
    'right_wing_up': right_wing_up_thruster_power,
    'right_wing_down': right_wing_down_thruster_power,
    'top_left': top_left_thruster_power,
    'top_right': top_right_thruster_power,
    'bottom_left': bottom_left_thruster_power,
    'bottom_right': bottom_right_thruster_power
}


######## AIR 
normal_n = 300
expiry_distance = 20
required_av_air = 800
pos_rand_factor=1
vel_rand_factor=0.5
standerd_speed = 0.01
generate_every = 5 # frames