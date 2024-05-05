import pygame
import sys
import numpy as np
import random

# Define colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)




normal_n = 300
expiry_distance = 20
required_av_air = 800
pos_rand_factor=1
vel_rand_factor=0.5
standerd_speed = 0.01
generate_every = 5 # frames






import pymunk
from pymunk.vec2d import Vec2d

class AirSphere:
    def __init__(self, size, mass, initial_position, velocity, expiry_distance=500):
        self.size = size
        self.mass = mass
        self.radius = size // 2
        self.expiry_distance = expiry_distance
        self.distance_traveled = 0

        # Create a pymunk Body and Shape for the AirSphere
        self.body = pymunk.Body(mass, pymunk.moment_for_circle(mass, 0, self.radius))
        self.body.position = Vec2d(*initial_position)
        self.body.velocity = Vec2d(*velocity)
        self.shape = pymunk.Circle(self.body, self.radius)
        self.shape.elasticity = 1.0  # Assuming perfect elasticity

    def update(self):
        # Update distance traveled
        self.distance_traveled += self.body.velocity.length

    def expire(self):
        # Check if air particle has traveled beyond expiry distance
        return self.distance_traveled >= self.expiry_distance





def create_air(space, n=1000, size=2, mass=0.01, expiry_distance=700, standard_speed=1.5, pos_rand_factor=1.5, vel_rand_factor=1.5, pos=(200, 200)):
    air_particles = []
    for _ in range(n):
        x_pos = pos[0] + (random.uniform(-0.5, 0.5) * pos_rand_factor * 100)
        y_pos = pos[1] + (random.uniform(-0.5, 0.5) * pos_rand_factor * 100)
        x_vel = standard_speed + (random.uniform(-0.5, 0.5) * vel_rand_factor)
        y_vel = random.uniform(-0.5, 0.5) * vel_rand_factor
        particle = AirSphere(size=size, mass=mass, initial_position=(x_pos, y_pos), velocity=(x_vel, y_vel), expiry_distance=expiry_distance)
        space.add(particle.body, particle.shape)
        air_particles.append(particle)
    return air_particles


# Initialize pymunk space
space = pymunk.Space()
space.gravity = (0, 0)  # Assuming no gravity for air particles

# Create air particles
air_particles = create_air(space)





























class AirSphere:
    def __init__(self, size, mass, initial_position, velocity,expiry_distance= 500):
        self.size = size
        self.mass = mass
        self.position = np.array(initial_position, dtype='float')
        self.velocity = np.array(velocity, dtype='float')
        self.radius = size // 2
        self.distance_traveled = 0  # Track the distance traveled by the air particle
        self.expiry_distance = expiry_distance  # Expiry distance for the air particle
        self.previous_position = np.array(initial_position, dtype='float')
        

    def draw(self, screen):
        pygame.draw.circle(screen, WHITE, (int(self.position[0]), int(self.position[1])), self.radius)

    def update(self):
        # Update position based on velocity
        self.position += self.velocity
        # Update distance traveled
        self.distance_traveled += np.linalg.norm(self.velocity)

    def check_collision(self, obj):
        # Calculate relative position and distance
        relative_position = obj.position - self.position
        distance = np.linalg.norm(relative_position)
        if distance <= self.radius + obj.radius:  # If distance is less than sum of radii, collision occurs
            # Calculate collision normal (direction from self to obj)
            collision_normal = relative_position / distance
            # Calculate relative velocity
            relative_velocity = self.velocity - obj.velocity
            # Calculate impulse scalar
            impulse_scalar = -2 * np.dot(relative_velocity, collision_normal) / (1 / self.mass + 1 / obj.mass)
            # Calculate impulse in collision normal direction
            impulse = impulse_scalar * collision_normal
            # Update velocities of both spheres
            self.velocity += impulse / self.mass
            obj.velocity -= impulse / obj.mass

    def check_mouse_interaction(self):
        global screen,screen_width,screen_height
        mouse_pos = pygame.mouse.get_pos()
        mouse_pressed = pygame.mouse.get_pressed()
        if mouse_pressed[0]:  # Left mouse button pressed
            # Calculate distance between mouse and center of the sphere
            distance_vector = np.array(mouse_pos) - self.position
            distance = np.linalg.norm(distance_vector)
            if distance <= self.radius:
                # Move the sphere to the mouse position
                self.position = np.array(mouse_pos)
                # Adjust position to prevent overlap
                self.position[0] = np.clip(self.position[0], self.radius, screen_width - self.radius)
                self.position[1] = np.clip(self.position[1], self.radius, screen_height - self.radius)

    def expire(self):
        # Check if air particle has traveled beyond expiry distance
        if self.distance_traveled >= self.expiry_distance:
            return True
        else:
            return False



def create_air(n=1000, size=2, mass=0.01,expiry_distance=700,standerd_speed = 1.5,pos_rand_factor = 1.5,vel_rand_factor = 1.5,pos=[200,200]):
    
    air_particles = []
    rand = np.random.random([n, 4])
    x_pos = pos[0]
    y_pos = pos[1]

    rand_pos_s = [x_pos+ (rand[:, 0] - 0.5)*pos_rand_factor * (100*np.power(n/300,0.15)), y_pos + (rand[:, 1] - 0.5)*pos_rand_factor * 100*np.power(n/300,0.15)]
    rand_vel = [standerd_speed + (rand[:, 2] - 0.5)*vel_rand_factor,  (rand[:, 3] - 0.5)*vel_rand_factor]
    # print(rand_pos_s)

    for i in range(n):
        air_particles.append(AirSphere(size=size, mass=mass, initial_position=[rand_pos_s[0][i], rand_pos_s[1][i]], velocity=[rand_vel[0][i], rand_vel[1][i]],expiry_distance=expiry_distance))

    return air_particles


def maintain_air(air, space, c, pos=(200, 200), generate_every=100, required_av_air=500,
                            normal_n=10, expiry_distance=700, vel_rand_factor=1.5, pos_rand_factor=1.5,
                            standerd_speed=1.5):
    # Update air particles and remove expired ones
    air[:] = [i for i in air if not i.expire()]
    
    # Add new air particles if necessary
    if not c % generate_every and len(air) < required_av_air:
        print(normal_n)
        new_air_particles = create_air(n=normal_n, expiry_distance=expiry_distance,
                                       vel_rand_factor=vel_rand_factor, pos_rand_factor=pos_rand_factor,
                                       standerd_speed=standerd_speed, pos=pos)
        for particle in new_air_particles:
            air.append(particle)
            space.add(particle.body, particle.shape)

    # Update each air particle
    for particle in air:
        particle.update()

    return air

            


def draw_fps(screen,clock):
    
    font = pygame.font.SysFont(None, 30)  # Choose font and font size
    fps = int(clock.get_fps())
    fps_text = f"FPS: {fps}"  # Get current FPS and convert to string
    if fps>30:
        text_surface = font.render(fps_text, True, pygame.Color('green'))  # Render FPS text
    elif fps>15:
        text_surface = font.render(fps_text, True, pygame.Color('yellow'))  # Render FPS text
    else:
        text_surface = font.render(fps_text, True, pygame.Color('red'))  # Render FPS text

    screen.blit(text_surface, (screen.get_width() - text_surface.get_width() - 10, 10))  # Draw FPS text at top right corner
    # print("drawn")
    return screen

def draw_text(text,screen,pos:list,size=30,color = 'white'):
    font = pygame.font.SysFont(None, size)  # Choose font and font size
    text_surface = font.render(text, True, pygame.Color(color))  # Render FPS text
    screen.blit(text_surface, pos)  # Draw FPS text at top right corner
    return screen



