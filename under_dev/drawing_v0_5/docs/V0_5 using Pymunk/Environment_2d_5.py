import pygame
import math
import numpy as np
import random
import air_more_realistic_f
import pymunk
import pymunk.pygame_util


from paras import *
import envi_utils


def handle_thrust_controls(spacecraft_body, keys_pressed, max_thrust):
    """
    Checks for keyboard clicks and updates thruster power levels based on user input.

    Args:
        space: The Pymunk physics space.
        spacecraft_body: The Pymunk body representing the spacecraft.
        keys_pressed: A dictionary containing the state of all pressed keys (True for pressed, False for not pressed).
        max_thrust: The maximum thrust power.

    Modifies:
        Applies forces to the thruster bodies in the Pymunk space.
    """
    # Define the force vectors for each thruster
    left_thruster_force = (0, max_thrust)
    right_thruster_force = (0, max_thrust)
    up_thruster_force = (0, max_thrust)
    down_thruster_force = (0, max_thrust)

    # Check keys and apply forces
    if keys_pressed[pygame.K_q]:  # 'Q' key for left wing up thruster
        spacecraft_body.apply_force_at_local_point(left_thruster_force, (-10, 5))
    if keys_pressed[pygame.K_a]:  # 'A' key for left wing down thruster
        spacecraft_body.apply_force_at_local_point(left_thruster_force, (-10, -5))
    if keys_pressed[pygame.K_e]:  # 'E' key for right wing up thruster
        spacecraft_body.apply_force_at_local_point(right_thruster_force, (10, 5))
    if keys_pressed[pygame.K_d]:  # 'D' key for right wing down thruster
        spacecraft_body.apply_force_at_local_point(right_thruster_force, (10, -5))

    if keys_pressed[pygame.K_j]:  # 'J' key for top left thruster
        spacecraft_body.apply_force_at_local_point(up_thruster_force, (0, 15))
    if keys_pressed[pygame.K_l]:  # 'L' key for top right thruster
        spacecraft_body.apply_force_at_local_point(up_thruster_force, (0, 15))
    if keys_pressed[pygame.K_u]:  # 'U' key for bottom left thruster
        spacecraft_body.apply_force_at_local_point(down_thruster_force, (0, -22))
    if keys_pressed[pygame.K_o]:  # 'O' key for bottom right thruster
        spacecraft_body.apply_force_at_local_point(down_thruster_force, (0, -22))








# Function to create a new Pymunk body with a polygon shape
def create_spacecraft_body(mass, moment, points):
    global space
    body = pymunk.Body(mass, moment)
    body.position = (300, 300)
    shape = pymunk.Poly(body, points)
    shape.color = pygame.color.THECOLORS["yellow"]
    space.add(body, shape)
    return body







pygame.display.set_caption("Mars Landing Simulation")
clock = pygame.time.Clock()

pygame.init()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
space = pymunk.Space()

# Simulation loop
running = True
landed = False
crashed = False
# input()
c= 0 
c_c1 = 0
c_c2 = 0

air =[]




space.gravity = (0, 0.01)  # Set gravity for the simulation
pymunk.pygame_util.positive_y_is_up = False

options = pymunk.pygame_util.DrawOptions(screen)


# Calculate the moment of inertia for the polygon shape
spacecraft_points = [(2, -8), (-2, -8), (-10, -2), (-20, 7), (-7, 15), (7, 15), (20, 7), (10, -2)]

spacecraft_moment = pymunk.moment_for_poly(spacecraft_mass, spacecraft_points)

# Create the spacecraft body with the calculated moment and the points defining the shape
spacecraft_body = create_spacecraft_body(spacecraft_mass, spacecraft_moment, spacecraft_points)


dt = clock.tick(60) / slower  # Time step in seconds

while running:
    c+=1

    air = air_more_realistic_f.maintain_air(c= c,space=space,air=air,pos=[x_position//SCALE,SCREEN_HEIGHT+(-y_position-10)//SCALE])

    mass = spacecraft_mass + fuel_mass

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    keys_pressed = pygame.key.get_pressed()

    

    envi_utils.apply_air_resistance_v2(dt) 
    envi_utils.apply_random_wind_effect(spacecraft_body= spacecraft_body, c= c)


    keys_pressed = pygame.key.get_pressed()

    handle_thrust_controls(keys_pressed= keys_pressed,spacecraft_body = spacecraft_body,max_thrust= max_thrust)


    envi_utils.apply_thrust(spacecraft_body, dt, max_thrust, control_ori_st, total_fuel_time, fuel_remaining)

    
    # envi_utils.update_position(dt)

    # Draw environment and spacecraft
    envi_utils.draw_environment(screen)
    spacecraft_x = int(x_position / SCALE)  # Convert x-position to screen coordinates
    spacecraft_y = SCREEN_HEIGHT - int(y_position / SCALE)  # Y-axis flipped for visual representation

    envi_utils.draw_thrusters_effect(screen, [spacecraft_x,spacecraft_y], spacecraft_orientation, thruster_powers)
    # print(spacecraft_x,spacecraft_y)

    
    air_more_realistic_f.draw_fps(screen,clock)
    

    left_vertical_thruster_power=   left_wing_up_thruster_power - left_wing_down_thruster_power 
    right_vertical_thruster_power= right_wing_up_thruster_power - right_wing_down_thruster_power
    up_horizontal_thruster_power = top_right_thruster_power - top_left_thruster_power
    down_horizontal_thruster_power = bottom_left_thruster_power - bottom_right_thruster_power

    # Draw speed text on top of the spacecraft
    envi_utils.draw_spacecraft_info(screen, spacecraft_x, spacecraft_y, x_velocity, y_velocity,spacecraft_orientation,
                         left_vertical_thruster_power,right_vertical_thruster_power,
                         up_horizontal_thruster_power,down_horizontal_thruster_power)
    



    # Check for landing or crash
    if y_position <= 500:  # Adjust based on landing area height
        if abs(y_velocity) < 20.0:  # Tolerable landing speed
            landed = True
            color = YELLOW
        else:
            crashed = True
            color = RED

    # Update display color based on landing/crash status
    if landed or crashed:
        screen.fill(color)  # Change background color

    # Display landing/crash message
    if landed:
        font = pygame.font.SysFont(None, 36)
        text = font.render("Successful Landing!", True, BLACK)
        text_rect = text.get_rect(center=(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2))
        screen.blit(text, text_rect)
    elif crashed:
        font = pygame.font.SysFont(None, 36)
        text = font.render("Crash Landing!", True, BLACK)
        text_rect = text.get_rect(center=(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2))
        screen.blit(text, text_rect)

    pygame.display.flip()  # Update the display

    # Exit the loop after landing or crash
    if landed or crashed:
        pygame.time.wait(3000)  # Wait for 3 seconds before quitting
        running = False

    space.debug_draw(options)
    space.step(dt)
    # screen.blit(text, (15, 15))
    pygame.display.flip()
    clock.tick(fps)

pygame.quit()