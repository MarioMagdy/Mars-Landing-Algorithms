import pygame
import sys
import numpy as np

# Initialize Pygame
pygame.init()

# Set the default font
pygame.font.init()
screen_width, screen_height = 800, 600
screen = pygame.display.set_mode((screen_width, screen_height))
clock = pygame.time.Clock()


# Define colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)



class AirSphere:
    def __init__(self, size, mass, initial_position, velocity,expiry_distance= 500):
        self.size = size
        self.mass = mass
        self.position = np.array(initial_position, dtype='float')
        self.velocity = np.array(velocity, dtype='float')
        self.radius = size // 2
        self.distance_traveled = 0  # Track the distance traveled by the air particle
        self.expiry_distance = expiry_distance  # Expiry distance for the air particle

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



def create_air(n=1000, size=2, mass=0.01,expiry_distance=700,standerd_speed = 1.5):
    air_particles = []
    rand = np.random.random([n, 4])
    x_pos = -200
    y_pos = 200

    rand_pos_s = [x_pos + (rand[:, 0] - 0.5)*4.5 * (100*np.power(n/300,0.15)), y_pos + (rand[:, 1] - 0.5)*4.5 * 100*np.power(n/300,0.15)]
    rand_vel = [standerd_speed + (rand[:, 2] - 0.5)*1.5,  (rand[:, 3] - 0.5)*1.5]

    for i in range(n):
        air_particles.append(AirSphere(size=size, mass=mass, initial_position=[rand_pos_s[0][i], rand_pos_s[1][i]], velocity=[rand_vel[0][i], rand_vel[1][i]],expiry_distance=expiry_distance))

    return air_particles



def draw_fps():
    font = pygame.font.SysFont(None, 30)  # Choose font and font size
    fps = int(clock.get_fps())
    fps_text = f"FPS: {fps}"  # Get current FPS and convert to string
    if fps>30:
        text_surface = font.render(fps_text, True, pygame.Color('green'))  # Render FPS text
    elif fps>15:
        text_surface = font.render(fps_text, True, pygame.Color('yellow'))  # Render FPS text
    else:
        text_surface = font.render(fps_text, True, pygame.Color('red'))  # Render FPS text

    screen.blit(text_surface, (screen_width - text_surface.get_width() - 10, 10))  # Draw FPS text at top right corner

def draw_text(text,pos:list,size=30,color = 'white'):
    font = pygame.font.SysFont(None, size)  # Choose font and font size
    text_surface = font.render(text, True, pygame.Color(color))  # Render FPS text
    screen.blit(text_surface, pos)  # Draw FPS text at top right corner



def colide(air_particle):
    air_particle.update()
    obstacle.check_collision(air_particle)
    air_particle.draw(screen)


class SquareObstacle:
    def __init__(self, size, mass, initial_position, velocity):
        self.size = size
        self.mass = mass
        self.position = np.array(initial_position, dtype='float')
        self.velocity = np.array(velocity, dtype='float')
        self.half_size = size // 2

    def draw(self, screen):
        pygame.draw.rect(screen, WHITE, (int(self.position[0] - self.size), int(self.position[1] - self.size), self.size, self.size))

    def update(self):
        # Update position based on velocity
        self.position += self.velocity

    def check_collision(self, air_particle):
        # Check for collision with the square obstacle
        left_edge = self.position[0] - self.half_size*2
        right_edge = self.position[0] + self.half_size*0.
        top_edge = self.position[1] - self.half_size*2
        bottom_edge = self.position[1] + self.half_size*0.

        # Check if the air particle is within the bounds of the square obstacle
        if left_edge - air_particle.radius <= air_particle.position[0] <= (right_edge + air_particle.radius) and \
           top_edge - air_particle.radius <= air_particle.position[1] <= (bottom_edge + air_particle.radius):
            # Calculate the angle of collision and adjust the impulse accordingly
            collision_angle = np.arctan2(air_particle.position[1] - self.position[1], air_particle.position[0] - self.position[0])
            collision_normal = np.array([np.cos(collision_angle), np.sin(collision_angle)])
            relative_velocity = air_particle.velocity - self.velocity
            impulse_scalar = -2 * np.dot(relative_velocity, collision_normal) / (1 / air_particle.mass + 1 / self.mass)
            impulse = impulse_scalar * collision_normal
            air_particle.velocity += impulse / air_particle.mass
            self.velocity -= impulse / self.mass

            # Reposition the air particle outside the square obstacle to prevent it from going through
            # Determine which edge the air particle collided with
            if air_particle.position[0] < left_edge:
                air_particle.position[0] = left_edge - air_particle.radius
            elif air_particle.position[0] > right_edge:
                air_particle.position[0] = right_edge + air_particle.radius
            if air_particle.position[1] < top_edge:
                air_particle.position[1] = top_edge - air_particle.radius
            elif air_particle.position[1] > bottom_edge:
                air_particle.position[1] = bottom_edge + air_particle.radius

            # Additional check for corner collisions
            if air_particle.position[0] < left_edge and air_particle.position[1] < top_edge:
                air_particle.position[0] = left_edge - air_particle.radius
                air_particle.position[1] = top_edge - air_particle.radius
            elif air_particle.position[0] > right_edge and air_particle.position[1] < top_edge:
                air_particle.position[0] = right_edge + air_particle.radius
                air_particle.position[1] = top_edge - air_particle.radius
            elif air_particle.position[0] < left_edge and air_particle.position[1] > bottom_edge:
                air_particle.position[0] = left_edge - air_particle.radius
                air_particle.position[1] = bottom_edge + air_particle.radius
            elif air_particle.position[0] > right_edge and air_particle.position[1] > bottom_edge:
                air_particle.position[0] = right_edge + air_particle.radius
                air_particle.position[1] = bottom_edge + air_particle.radius
                

    def check_mouse_interaction(self):
        mouse_pos = pygame.mouse.get_pos()
        mouse_pressed = pygame.mouse.get_pressed()
        if mouse_pressed[0]:  # Left mouse button pressed
            # Calculate distance between mouse and center of the sphere
            distance_vector = np.array(mouse_pos) - self.position
            distance = np.linalg.norm(distance_vector)
            if distance <= self.size:
                # Move the sphere to the mouse position
                self.position = np.array(mouse_pos,dtype='float')
                # Adjust position to prevent overlap
                self.position[0] = np.clip(self.position[0], self.size, screen_width - self.size,dtype='float')
                self.position[1] = np.clip(self.position[1], self.size, screen_height - self.size,dtype='float')


normal_n = 500
expiry_distance = 1200
required_av_air = 3000


# Create an obstacle and air particles
obstacle = SquareObstacle(size=170, mass=100, initial_position=[500, 500], velocity=[-0.0, -0.2])
air = create_air(n=normal_n,expiry_distance=expiry_distance,standerd_speed=2)



v_colide = np.vectorize(colide)



c=0
done = False
screen.fill(BLACK)

# Main game loop (unchanged)
while True:
    c+=1
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
    
    

    screen.fill(BLACK)

    obstacle.update()

    for i in air:
        colide(i)

    # Check if air particle has expired and remove it from the list
        if i.expire():
            air.remove(i)
            
    if not c%40:
        if len(air)<required_av_air:
            for i in create_air(n=normal_n,expiry_distance=expiry_distance):
                air.append(i)

    obstacle.check_mouse_interaction()


    vel = np.round(obstacle.velocity,3)

    draw_text(f'Object Velocity: {vel}',[20,screen_height-30])
    


    # Draw objects
    obstacle.draw(screen)
    draw_fps()
    draw_text(f"Number of Air Particles: {len(air)}",(screen_width-300,screen_height-30))

    # Refresh display
    pygame.display.flip()

    # Cap the frame rate
    clock.tick(60)