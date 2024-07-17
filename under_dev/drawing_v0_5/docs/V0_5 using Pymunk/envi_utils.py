import pygame
import math
import numpy as np
import random
import air_more_realistic_f
import pymunk
import pymunk.pygame_util

from paras import *

clock = pygame.time.Clock()

dt = clock.tick(60) / slower  # Time step in seconds

def update_position(dt):
    global x_position, y_position
    x_position += x_velocity * dt
    y_position += y_velocity * dt



def rotate_point(cx, cy, x, y, angle):
    sin_angle = math.sin(angle)
    cos_angle = math.cos(angle)
    
    # Translate point to origin
    x -= cx
    y -= cy
    
    # Rotate point
    x_new = x * cos_angle - y * sin_angle
    y_new = x * sin_angle + y * cos_angle
    
    # Translate point back
    x_new += cx
    y_new += cy
    
    return x_new, y_new



def draw_stars(screen,amount=3):

        # Draw stars in the background
        for _ in range(amount):
            star_x = np.random.randint(0, screen.get_width())
            star_y = np.random.randint(0, screen.get_height())
            pygame.draw.circle(screen, WHITE, (star_x, star_y), 1)





def draw_environment(screen):
    global time_from_start, dt
    """
    Draws an enhanced Martian environment on the Pygame screen.

    Customize this function to create a visually appealing representation.
    Suggestions:
        - Use an image for the Martian surface (texture).
        - Add layers of atmosphere (semi-transparent) and texture.
        - Incorporate Martian features like craters and mountains.
    """
    screen.fill(BLACK)  # Fill the background with black
    time_from_start+=dt
    current_height = y_position

    draw_stars(screen,2)

    
    # Load the Mars surface texture (replace 'mars_texture.png' with your image file)
    mars_texture = pygame.image.load(r'under_dev\drawing_v0_4\docs\mars_texture.png').convert_alpha()
    mars_background = pygame.image.load(r'under_dev\drawing_v0_4\docs\mars_background.png').convert_alpha()
    mars_background = pygame.transform.scale(mars_background, (SCREEN_WIDTH,SCREEN_HEIGHT))
 
    # Create a copy of the texture with adjusted opacity (60%)
    textured_surface = mars_texture.copy()
    textured_surface.set_alpha(120)  # 60% opacity (255 * 0.6)
    

    # Draw a solid rectangle for the Martian ground color
    pygame.draw.rect(screen, MARS_SURFACE_COLOR, (0, screen.get_height() - 50, screen.get_width(), 50))

    screen.blit(textured_surface, ( 0,screen.get_height() - 50))
    screen.blit(mars_background, ( 0,0))
    # Display mission information (time from start and current height)
    font = pygame.font.Font(None, 24)
    info_text = f"Time: {time_from_start:.2f} s  |  Height: {current_height/1000:.3f} km"
    text_surface = font.render(info_text, True, GREY)
    screen.blit(text_surface, (10, 10))

    # Draw a scale (ruler) for pixels to meters
    scale_length_pixels = 100
    scale_length_meters = SCALE*scale_length_pixels/1000
    scale_start_x = 10
    scale_end_x = scale_start_x + scale_length_pixels
    pygame.draw.line(screen, WHITE, (scale_start_x, 30), (scale_end_x, 30), 2)
    font_small = pygame.font.Font(None, 20)
    scale_label = f"{scale_length_meters} km"
    label_surface = font_small.render(scale_label, True, GREY)
    screen.blit(label_surface, (scale_start_x, 35))



    
    draw_ruler(screen)


def draw_ruler(screen):
    scale_x = screen.get_width() - 20
    scale_y_start = 20
    scale_y_end = screen.get_height() - 50
    pygame.draw.line(screen, GREY, (scale_x-50, scale_y_start), (scale_x-50, scale_y_end), 2)
    font_size = 18

    # Add labeled intervals to the scale
    font_small = pygame.font.Font(None, font_size)
    for i_pixel in range(50, SCREEN_HEIGHT-scale_y_start-30,50):
        # print(i_pixel)
        label = f"{i_pixel*SCALE//1000} km"
        label_surface = font_small.render(label, True, GREY)
        pygame.draw.line(screen, GREY,( (scale_x - 42), (scale_y_end - i_pixel+font_size//3)),
                         ( (scale_x - 50), (scale_y_end - i_pixel+font_size//3)) , 1)
        screen.blit(label_surface, ((scale_x - 40), (scale_y_end - i_pixel) ))
        



def apply_random_wind_effect(spacecraft_body, c, n_of_itter=5, random_wind_factor=2.0):
    """Add a small random change in the orintaion, 
    soon will also add it to the velocity distribution"""

    global orientation_change,spacecraft_orientation
    rotation = orientation_change
    if not c % n_of_itter:
        rotation = (random.random() - 0.5) * random_wind_factor
        spacecraft_body.apply_impulse_at_local_point((0, rotation))




def apply_air_resistance_v2(dt):
    """
    Calculates and applies air resistance force to each component of the spacecraft's velocity.

    Args:
        dt: Timestep (seconds).
        density: The air density (kg/m^3). Defaults to 0.02 (approximate Martian surface density).
    """
    global y_velocity ,x_velocity,y_position

    density = get_mars_atmospheric_density(y_position)*3

    # Adjust drag coefficients based on Mach number (placeholder values)
    drag_coefficient_x = calculate_drag_coefficient(x_velocity / 240)  # Adjust for Mars speed of sound
    drag_coefficient_y = calculate_drag_coefficient(y_velocity / 240)
    # Calculate air resistance forces
    air_resistance_force_x = -0.5 * density * drag_coefficient_x * np.pi * (x_velocity**2) *AIR_RESISTANCE_COEFF 
    air_resistance_force_y = 0.5 * density * drag_coefficient_y * np.pi * (y_velocity**2) * AIR_RESISTANCE_COEFF # cos bank angle

    # Update velocity components
    x_velocity += air_resistance_force_x * dt / spacecraft_mass 
    y_velocity += air_resistance_force_y * dt / spacecraft_mass

    return np.array([x_velocity, y_velocity])




def get_mars_atmospheric_density(altitude):
    """
    Calculates the atmospheric density on Mars at a given altitude.

    Args:
        altitude: Altitude in meters.

    Returns:
        Density in kg/m^3.
    """
    # Define the altitude ranges for the lower and upper atmosphere
    lower_atmosphere_altitude = 7000  # meters

    # Calculate temperature and pressure based on altitude
    if altitude <= lower_atmosphere_altitude:
        # Lower atmosphere (surface to 7,000 meters)
        temperature = -31 - 0.000998 * altitude
        pressure = 0.699 * np.exp(-0.00009 * altitude)
    else:
        # Upper atmosphere (above 7,000 meters)
        temperature = -23.4 - 0.00222 * altitude
        pressure = 0.699 * np.exp(-0.00009 * altitude)

    # Calculate density using the ideal gas law
    density = pressure / (0.1921 * (temperature + 273.1))

    return density





def draw_spacecraft_info(screen, x, y, x_velocity, y_velocity, orientation, left_thruster, 
                         right_thruster, up_thruster, down_thruster, font_size=16, font_color=WHITE):
    """
    Draws text displaying the spacecraft's velocities, orientation, and thruster levels.

    Args:
        screen: The Pygame display surface.
        x: The x-coordinate of the spacecraft's center.
        y: The y-coordinate of the spacecraft's center.
        x_velocity: The spacecraft's x-axis velocity (m/s).
        y_velocity: The spacecraft's y-axis velocity (m/s).
        orientation: The spacecraft's current orientation (degrees).
        left_thruster: The power level of the left vertical thruster (%).
        right_thruster: The power level of the right vertical thruster (%).
        up_thruster: The power level of the up horizontal thruster (%).
        down_thruster: The power level of the down horizontal thruster (%).
        font_size: The size of the font used to display the text (default: 16).
        font_color: The color of the text (default: WHITE).
    """
    font = pygame.font.SysFont(None, font_size)

    # Format the text for velocity, orientation, and thruster levels
    x_velocity_text = f"X-Velocity: {x_velocity:.1f} m/s"
    y_velocity_text = f"Y-Velocity: {y_velocity:.1f} m/s"
    orientation_text = f"Orientation: {-orientation:.1f}°"
    left_thruster_text = f"Left Thrust: {left_thruster}%"
    right_thruster_text = f"Right Thrust: {right_thruster}%"
    up_thruster_text = f"Up Thrust: {up_thruster}%"
    down_thruster_text = f"Down Thrust: {down_thruster}%"

    # Render the text surfaces
    x_text_surface = font.render(x_velocity_text, True, font_color)
    y_text_surface = font.render(y_velocity_text, True, font_color)
    orientation_surface = font.render(orientation_text, True, font_color)
    left_thruster_surface = font.render(left_thruster_text, True, font_color)
    right_thruster_surface = font.render(right_thruster_text, True, font_color)
    up_thruster_surface = font.render(up_thruster_text, True, font_color)
    down_thruster_surface = font.render(down_thruster_text, True, font_color)

    # Get the text surface dimensions for positioning
    x_text_width, x_text_height = x_text_surface.get_size()
    y_text_width, y_text_height = y_text_surface.get_size()
    orientation_width, orientation_height = orientation_surface.get_size()
    left_thruster_width, left_thruster_height = left_thruster_surface.get_size()
    right_thruster_width, right_thruster_height = right_thruster_surface.get_size()
    up_thruster_width, up_thruster_height = up_thruster_surface.get_size()
    down_thruster_width, down_thruster_height = down_thruster_surface.get_size()

    # Place the text surfaces below the spacecraft with some offset
    text_x = x - max(x_text_width, y_text_width, orientation_width, left_thruster_width, 
                     right_thruster_width, up_thruster_width, down_thruster_width) // 2
    text_y = y + x_text_height + 20  # Position the text slightly below the spacecraft

    # Blit the text surfaces onto the screen
    screen.blit(x_text_surface, (text_x, text_y))
    screen.blit(y_text_surface, (text_x, text_y + y_text_height))
    screen.blit(orientation_surface, (text_x, text_y + 2 * y_text_height))
    screen.blit(left_thruster_surface, (text_x, text_y + 3 * y_text_height))
    screen.blit(right_thruster_surface, (text_x, text_y + 4 * y_text_height))
    screen.blit(up_thruster_surface, (text_x, text_y + 5 * y_text_height))
    screen.blit(down_thruster_surface, (text_x, text_y + 6 * y_text_height))






def apply_thrust(spacecraft_body, dt, max_thrust, control_ori_st, total_fuel_time, fuel_remaining):
    global left_wing_up_thruster_power, left_wing_down_thruster_power
    global right_wing_up_thruster_power, right_wing_down_thruster_power
    global top_left_thruster_power, top_right_thruster_power
    global bottom_left_thruster_power, bottom_right_thruster_power
    # Assuming thruster powers are attributes of the spacecraft_body for easy access
    left_wing_vertical_force = (left_wing_up_thruster_power - left_wing_down_thruster_power) / 100.0 * max_thrust
    right_wing_vertical_force = (right_wing_up_thruster_power - right_wing_down_thruster_power) / 100.0 * max_thrust
    top_horizontal_force = (top_right_thruster_power - top_left_thruster_power) / 100.0 * max_thrust
    bottom_horizontal_force = (bottom_right_thruster_power - bottom_left_thruster_power) / 100.0 * max_thrust

    # Calculate the net force differences for rotation and movement
    net_vertical_force = right_wing_vertical_force + left_wing_vertical_force
    net_horizontal_force = top_horizontal_force + bottom_horizontal_force

    # Apply the forces to the spacecraft body
    spacecraft_body.apply_force_at_local_point((net_horizontal_force, net_vertical_force))

    # Apply rotation based on the net force differences
    rotation_rate = (right_wing_vertical_force - left_wing_vertical_force + bottom_horizontal_force - top_horizontal_force) * dt
    spacecraft_body.torque -= rotation_rate * control_ori_st

    # Fuel consumption is based on the total thrust exerted by all thrusters
    total_thrust = (abs(left_wing_vertical_force) + abs(right_wing_vertical_force) +
                    abs(top_horizontal_force) + abs(bottom_horizontal_force))
    fuel_remaining -= total_thrust / total_fuel_time

    # Return the updated fuel remaining
    return fuel_remaining
    


def calculate_drag_coefficient(mach_number):
    """
    Placeholder function to calculate drag coefficient based on Mach number.
    Replace with your actual implementation.
    """
    if mach_number <= 1:
        return 1.0  # Drag coefficient for subsonic speeds
    else:
        return 0.5  # Simplified drag coefficient for supersonic speeds



def update_orientation(spacecraft_body,dt):
    global spacecraft_orientation
    global max_thrust
    global left_wing_up_thruster_power, left_wing_down_thruster_power
    global right_wing_up_thruster_power, right_wing_down_thruster_power
    global top_left_thruster_power, top_right_thruster_power
    global bottom_left_thruster_power, bottom_right_thruster_power

    # Calculate the net force differences for rotation
    net_vertical_force_difference = (right_wing_up_thruster_power - right_wing_down_thruster_power) - \
                                    (left_wing_up_thruster_power - left_wing_down_thruster_power)
    net_horizontal_force_difference = (bottom_right_thruster_power - bottom_left_thruster_power) - \
                                    (top_right_thruster_power - top_left_thruster_power)

    # Calculate the orientation change based on the net force differences
    # The multipliers can be adjusted to represent the sensitivity of the spacecraft's rotation
    orientation_change = (net_vertical_force_difference + net_horizontal_force_difference) * dt * 0.05  # Arbitrary value for rotation sensitivity

    net_torque = orientation_change * dt * 0.05  # Adjust the multiplier as needed

    spacecraft_body.apply_torque(net_torque)






def draw_thrusters_effect(screen, spacecraft_position, spacecraft_orientation, thruster_powers):
    orientation_radians = math.radians(spacecraft_orientation)
    effect_length = 0.35  # Base length of the thruster effect

    # Define the offsets and angles for each thruster from the spacecraft's center
    thruster_info = {
        'left_wing_up': {'offset': (-10, 5), 'angle': 90},
        'left_wing_down': {'offset': (-10, 5), 'angle': -90},
        'right_wing_up': {'offset': (10, 5), 'angle': 90},
        'right_wing_down': {'offset': (10, 5), 'angle': -90},
        'top_left': {'offset': (0, 15), 'angle': 180},
        'top_right': {'offset': (0, 15), 'angle': 0},
        'bottom_left': {'offset': (0, -22), 'angle': 180},
        'bottom_right': {'offset': (0, -22), 'angle': 0}
    }

    for thruster, power in thruster_powers.items():
        if power > 0:  # Draw the effect only if the thruster is active
            info = thruster_info[thruster]
            offset_x, offset_y = info['offset']
            angle_offset = info['angle']
            start_pos = (
                spacecraft_position[0] + offset_x,
                spacecraft_position[1] + offset_y
            )
            rotated_offset_x, rotated_offset_y = rotate_point(0, 0, offset_x, offset_y, orientation_radians)
            start_pos = (
                spacecraft_position[0] + rotated_offset_x,
                spacecraft_position[1] + rotated_offset_y
            )
            end_pos = (
                start_pos[0] + effect_length * power * math.cos(orientation_radians + math.radians(angle_offset)),
                start_pos[1] + effect_length * power * math.sin(orientation_radians + math.radians(angle_offset))
            )
            pygame.draw.line(screen, THRUSTER_EFFECT_COLOR, start_pos, end_pos, 2)
