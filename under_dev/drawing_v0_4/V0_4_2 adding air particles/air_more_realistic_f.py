import pygame
import sys
import numpy as np


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




def maintain_air(c,air,pos=[200,200]):
    if not c%generate_every:
        if len(air)<required_av_air:
            for i in create_air(n=normal_n,expiry_distance=expiry_distance,vel_rand_factor=vel_rand_factor,pos_rand_factor=pos_rand_factor,standerd_speed=standerd_speed,pos=pos):
                air.append(i)

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


def update_air(air,c):
    for i in air:
        i.update()
        if i.expire():
            air.remove(i)

    





def line_intersects(p1, p2, start_point, end_point):
    # This function will check if the line segment from start_point to end_point
    # intersects with the line segment from p1 to p2

    def ccw(A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    # Check if the lines are intersecting
    return ccw(p1, start_point, end_point) != ccw(p2, start_point, end_point) and \
           ccw(p1, p2, start_point) != ccw(p1, p2, end_point)

# Now you can use this function in your check_trajectory function



def rotate_point(cx, cy, x, y, angle):
    # Rotate a point counterclockwise by a given angle around a given origin.
    s, c = np.sin(angle), np.cos(angle)
    # Translate point back to origin
    x -= cx
    y -= cy
    # Rotate point
    new_x = x * c - y * s
    new_y = x * s + y * c
    # Translate point back
    x = new_x + cx
    y = new_y + cy
    return x, y



def check_collision_spacecraft(air_particle, spacecraft_points):
    # Function to calculate the edge normal
    def edge_normal(p1, p2):
        edge = (p2[0] - p1[0], p2[1] - p1[1])
        normal = (-edge[1], edge[0])
        return normal

    # Function to calculate the dot product of two vectors
    def dot_product(v1, v2):
        return sum((a*b) for a, b in zip(v1, v2))

    # Function to project a polygon onto an axis
    def project_polygon(axis, polygon):
        dots = [dot_product(vertex, axis) for vertex in polygon]
        return [min(dots), max(dots)]

    # Function to calculate the distance between two intervals
    def interval_distance(minA, maxA, minB, maxB):
        if minA < minB:
            return minB - maxA
        else:
            return minA - maxB
        
    # def distance(p1, p2):
    #     return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

        
    # def is_point_inside_polygon(point, polygon):
    #     # Cast a ray from the point to the right
    #     count = 0
    #     for i in range(len(polygon)):
    #         start = polygon[i]
    #         end = polygon[(i + 1) % len(polygon)]
    #         if start[1] > point[1] != end[1] > point[1] and \
    #                 point[0] < (end[0] - start[0]) * (point[1] - start[1]) / (end[1] - start[1]) + start[0]:
    #             count += 1
    #     # Point is inside if count is odd
    #     return count % 2 == 1

    # # Check if the air particle is inside the spacecraft
    # if is_point_inside_polygon(air_particle.position, spacecraft_points):
    #     # Find the closest point on the spacecraft's body to move the particle outside
    #     closest_point = min(spacecraft_points, key=lambda p: distance(air_particle.position, p))
    #     air_particle.position = closest_point
    #     return True


    # Function to check the particle's trajectory for potential collisions
    def check_trajectory(air_particle, spacecraft_points):
        start_point = air_particle.previous_position
        end_point = air_particle.position
        trajectory_vector = (end_point[0] - start_point[0], end_point[1] - start_point[1])

        for i in range(len(spacecraft_points)):
            p1 = spacecraft_points[i]
            p2 = spacecraft_points[(i + 1) % len(spacecraft_points)]
            if line_intersects(p1, p2, start_point, end_point):
                return True
        return False

    # Build a list of the spacecraft's edges' normals
    normals = []
    for i in range(len(spacecraft_points)):
        p1 = spacecraft_points[i]
        p2 = spacecraft_points[(i + 1) % len(spacecraft_points)]
        normals.append(edge_normal(p1, p2))

    # Check for separation on each axis
    for normal in normals:
        min_max_air = [dot_product(air_particle.position, normal) - air_particle.radius,
                       dot_product(air_particle.position, normal) + air_particle.radius]
        projections = [dot_product(point, normal) for point in spacecraft_points]
        min_max_spacecraft = [min(projections), max(projections)]

        distance = interval_distance(min_max_air[0], min_max_air[1], min_max_spacecraft[0], min_max_spacecraft[1])
        if distance > 0:
            return False

    # If no separating axis found, check the trajectory for potential collisions
    if check_trajectory(air_particle, spacecraft_points):
        air_particle.velocity = -air_particle.velocity
        return True

    return False
