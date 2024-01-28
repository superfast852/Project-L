import pygame
import math
import numpy as np
from time import time
from extensions.NavStack import Map

# Initialize pygame
pygame.init()
mapgen = Map("random")

norm2pg = lambda arr: np.logical_not(np.flipud(np.rot90(arr))).astype(int)
empty = np.zeros((mapgen.map.shape[0], mapgen.map.shape[1]))+1
map_array = norm2pg(mapgen.map)
print(map_array.shape)


def cast_ray(start_pos, angle, max_distance):
    count = 0
    bases = (math.cos(angle), math.sin(angle))
    while True:
        x, y = start_pos[0] + (count * bases[0]), start_pos[1] + (count * bases[1])
        if x < 0 or y < 0 or x > SCREEN_WIDTH or y > SCREEN_HEIGHT:
            return None
        try:
            if map_array[int(y), int(x)] == 0:
                return x, y
        except IndexError:
            pass
        count += 1
        if count > max_distance:
            return None


# Colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLACK = (0, 0, 0)

# Screen dimensions
SCREEN_HEIGHT, SCREEN_WIDTH = map_array.shape


# Initial robot position and orientation
robot_x = SCREEN_WIDTH // 2
robot_y = SCREEN_HEIGHT // 2
robot_orientation = 0  # Angle in radians


# Create a display screen
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption('Robot Simulation')

# Initialize clock for controlling FPS
clock = pygame.time.Clock()
FPS = 120  # Target frames per second
target_position = None
angle = math.pi/4
line = True
permanence = True
running = True
start = time()
frame_counter = 0
while running:
    frame_counter += 1
    if not permanence:
        if frame_counter % 10 == 0:
            empty = np.zeros((mapgen.map.shape[0], mapgen.map.shape[1]))+1
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    if frame_counter % 15 == 0:
        frame_counter = 0
        keys = pygame.key.get_pressed()
        if keys[pygame.K_SPACE]:
            line = not line
        if keys[pygame.K_r]:
            empty = np.zeros((mapgen.map.shape[0], mapgen.map.shape[1]))+1
        if keys[pygame.K_p]:
            permanence = not permanence
    # Clear the screen
    screen.fill(WHITE)
    screen.blit(pygame.surfarray.make_surface(norm2pg(empty)), (0, 0))
    mouse_position = pygame.mouse.get_pos()

    # line casting test
    start = time()
    for i in range(0, 360, 2):
        point = cast_ray(mouse_position, math.radians(i), 2000)
        if point:
            if line:
                pygame.draw.line(screen, RED, mouse_position, point, 1)
            empty[int(point[1]), int(point[0])] = 0
    #print(1 / (time() - start))
    # Update the display
    pygame.display.flip()

    # Cap the frame rate
    clock.tick(FPS)


pygame.quit()