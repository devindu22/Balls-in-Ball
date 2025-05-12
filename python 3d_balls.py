import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import random
import math
import numpy as np

# Constants for the simulation
WIDTH = 800          # Screen width in pixels
HEIGHT = 800         # Screen height in pixels
R = 400              # Radius of the spherical boundary
BALL_RADIUS = 10      # Visual radius of each ball
NUM_BALLS = 400      # Number of balls in the simulation
VMAX = 400           # Maximum speed of balls in pixels per second
FPS = 60             # Target frames per second

# Initialize Pygame and OpenGL
pygame.init()
display = (WIDTH, HEIGHT)
pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
clock = pygame.time.Clock()

# Set up the OpenGL perspective
gluPerspective(45, (WIDTH / HEIGHT), 0.1, 2000.0)
glTranslatef(0.0, 0.0, -1000)  # Move camera back to see the sphere

# Enable depth testing for 3D rendering
glEnable(GL_DEPTH_TEST)

# Ball class for 3D
class Ball:
    def __init__(self):
        # Generate random initial position inside the sphere
        theta = random.uniform(0, 2 * math.pi)
        phi = random.uniform(0, math.pi)
        r = R * (random.uniform(0, 1) ** (1/3))  # Cube root for uniform distribution in 3D
        x = r * math.sin(phi) * math.cos(theta)
        y = r * math.sin(phi) * math.sin(theta)
        z = r * math.cos(phi)
        self.pos = np.array([x, y, z])

        # Generate random velocity
        speed = random.uniform(0, VMAX)
        theta_v = random.uniform(0, 2 * math.pi)
        phi_v = random.uniform(0, math.pi)
        vx = speed * math.sin(phi_v) * math.cos(theta_v)
        vy = speed * math.sin(phi_v) * math.sin(theta_v)
        vz = speed * math.cos(phi_v)
        self.vel = np.array([vx, vy, vz])

        # Random color
        self.color = (
            random.randint(0, 255) / 255.0,
            random.randint(0, 255) / 255.0,
            random.randint(0, 255) / 255.0
        )

    def update(self, dt):
        # Compute potential new position
        p1 = self.pos + self.vel * dt
        dist = np.linalg.norm(p1)

        if dist <= R:
            self.pos = p1
        else:
            # Find time of intersection with sphere
            a = np.dot(self.vel, self.vel)
            b = 2 * np.dot(self.pos, self.vel)
            c = np.dot(self.pos, self.pos) - R**2
            discriminant = b**2 - 4 * a * c

            if discriminant >= 0:
                sqrt_disc = math.sqrt(discriminant)
                t1 = (-b - sqrt_disc) / (2 * a)
                t2 = (-b + sqrt_disc) / (2 * a)
                t_candidates = [t for t in [t1, t2] if t > 0]
                if t_candidates:
                    t = min(t_candidates)
                    if t <= dt:
                        # Move to collision point
                        pc = self.pos + self.vel * t
                        # Compute normal
                        n = pc / np.linalg.norm(pc)
                        # Reflect velocity
                        v_dot_n = np.dot(self.vel, n)
                        v_reflect = self.vel - 2 * v_dot_n * n
                        # Update position
                        self.pos = pc + v_reflect * (dt - t)
                        # Ensure inside sphere
                        if np.linalg.norm(self.pos) > R:
                            self.pos = (self.pos / np.linalg.norm(self.pos)) * R
                        self.vel = v_reflect

    def draw(self):
        glPushMatrix()
        glTranslatef(self.pos[0], self.pos[1], self.pos[2])
        glColor3fv(self.color)
        quad = gluNewQuadric()
        gluSphere(quad, BALL_RADIUS, 20, 20)
        glPopMatrix()

# Create balls
balls = [Ball() for _ in range(NUM_BALLS)]

# Function to draw the wireframe sphere
def draw_sphere():
    glColor3f(1.0, 1.0, 1.0)
    quad = gluNewQuadric()
    gluQuadricDrawStyle(quad, GLU_LINE)
    gluSphere(quad, R, 20, 20)

# Main loop
running = True
angle = 0
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    dt = clock.tick(FPS) / 1000.0

    # Update balls
    for ball in balls:
        ball.update(dt)

    # Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    # Rotate the view
    glPushMatrix()
    glRotatef(angle, 0, 1, 0)  # Rotate around y-axis
    angle += 1

    # Draw sphere boundary
    draw_sphere()

    # Draw balls
    for ball in balls:
        ball.draw()

    glPopMatrix()

    pygame.display.flip()

pygame.quit()