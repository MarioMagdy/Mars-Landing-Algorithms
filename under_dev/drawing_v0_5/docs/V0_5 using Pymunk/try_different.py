import pygame
import pymunk
import pymunk.pygame_util
import random
import numpy

# Pygame setup
pygame.init()
width, height = 600, 600
screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()
draw_options = pymunk.pygame_util.DrawOptions(screen)

# Pymunk setup
space = pymunk.Space()
space.gravity = (0, 0)  # No gravity in space

# Spacecraft body
spacecraft_body = pymunk.Body(1, numpy.inf)
spacecraft_body.position = (300, 300)
spacecraft_shape = pymunk.Circle(spacecraft_body, 30)
spacecraft_shape.color = pygame.color.THECOLORS["yellow"]
space.add(spacecraft_body, spacecraft_shape)



def apply_air_resistance(spacecraft_body, air_particles):
    for particle_shape in air_particles:
        offset = pymunk.Vec2d(*(particle_shape.body.position - spacecraft_body.position))
        distance = offset.length
        # Simple model for air resistance: inversely proportional to the square of the distance
        force_magnitude = 1 / (distance ** 2 + 1)  # +1 to avoid division by zero
        force = offset.normalized() * force_magnitude
        spacecraft_body.apply_force_at_local_point(-force)

# Game loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Apply air resistance
    apply_air_resistance(spacecraft_body, space.shapes)

    # Update physics
    space.step(1/50)
    screen.fill((0, 0, 0))  # Space is black
    space.debug_draw(draw_options)

    pygame.display.flip()
    clock.tick(50)

pygame.quit()
