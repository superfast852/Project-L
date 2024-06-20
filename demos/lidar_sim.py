import pygame
import math
import numpy as np
from time import time
from extensions.NavStack_Normed import Map
from extensions.tools import line2dots
from networktables import NetworkTables
from numba import njit, jit

# Initialize pygame
pygame.init()
mapgen = Map("random")
NetworkTables.initialize("127.0.0.1")
meters = 35

maptable = NetworkTables.getTable("map")
while maptable.getNumber("meters", 0) == 0:
    maptable.putNumber("meters", meters)

botTable = NetworkTables.getTable("bot")
while botTable.getNumber("size", 0) == 0:
    botTable.putNumber("size", meters/3.5)

flatres = mapgen.map.shape[0]**2
norm2pg = lambda arr: np.flipud(np.rot90(arr))
# Changed to -1 as undiscovered
empty = (np.zeros((mapgen.map.shape[0], mapgen.map.shape[1]))-1).astype(int)
map_array = norm2pg(mapgen.map).T


def cast_ray(start_pos, angle, max_distance):
    count = 0
    bases = (math.cos(angle), math.sin(angle))
    while True:
        x, y = round(start_pos[0] + (count * bases[0])), round(start_pos[1] + (count * bases[1]))
        x = x if x>0 else 0
        x = x if x<SCREEN_WIDTH else SCREEN_WIDTH-1
        y = y if y > 0 else 0
        y = y if y < SCREEN_HEIGHT else SCREEN_HEIGHT-1
        try:
            if map_array[y, x] == 1 or x == 0 or y == 0 or x == SCREEN_WIDTH-1 or y == SCREEN_HEIGHT-1:
                return x, y
        except IndexError:
            pass
        count += 1
        if count > max_distance:
            return None


@njit()
def npcast_ray(start_pos, angle, max_distance):
    # Create a NumPy array for the start position
    start = np.array(start_pos)

    # Calculate the step vector
    step = np.array([np.cos(angle), np.sin(angle)])

    # Generate a range of steps
    steps = np.arange(max_distance)

    # Calculate all positions along the ray
    x_positions = start[0] + steps * step[0]
    y_positions = start[1] + steps * step[1]

    # Clip positions to stay within the screen boundaries
    x_positions = np.clip(x_positions, 0, SCREEN_WIDTH - 1)
    y_positions = np.clip(y_positions, 0, SCREEN_HEIGHT - 1)

    # Round positions to nearest integer
    x_positions = np.round(x_positions).astype(np.int16)
    y_positions = np.round(y_positions).astype(np.int16)

    # Check for collision with map objects
    for x, y in zip(x_positions, y_positions):
        if map_array[y, x] == 1  or x == 0 or y == 0 or x == SCREEN_WIDTH - 1 or y == SCREEN_HEIGHT - 1:
            return x, y

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
FPS = 60  # Target frames per second

line = True
running = True
auto = False
pose = np.array([17.5, 17.5, 360*np.random.random()])
prevtime = time()

start = time()
frame_counter = 0
while running:
    start = time()
    frame_counter += 1
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    if frame_counter % 15 == 0:
        frame_counter = 0
        keys = pygame.key.get_pressed()
        if keys[pygame.K_SPACE]:
            line = not line
        if keys[pygame.K_r]:
            empty = np.zeros((mapgen.map.shape[0], mapgen.map.shape[1])).astype(int)-1

    # Clear the screen
    screen.fill(WHITE)
    screen.blit(pygame.surfarray.make_surface(norm2pg(empty)), (0, 0))
    if auto:
        currtime = time()

        s = currtime - prevtime
        theta = np.radians(pose[2])
        pose[0] += s * np.cos(theta)
        pose[1] += s * np.sin(theta)
        pose[2] += 5 * np.random.randn()
        prevtime = currtime
        mouse_position = mapgen.m2px(pose)[:2]

    else:
        mouse_position = pygame.mouse.get_pos()

    mouse_position = tuple(np.clip(mouse_position, 0, 799))

    ts = time()
    try:
        if map_array[mouse_position[::-1]] == 0:
            for i in (np.linspace(0, 2*np.pi, 180) + np.random.normal(0, 0.005, 180)):
                point = npcast_ray(mouse_position, i, 1100)
                if point:
                    if line:
                        pygame.draw.line(screen, RED, mouse_position, point, 1)

                    for dot in line2dots(*tuple([mouse_position[::-1], point[::-1]])):
                        empty[dot] = 0
                    empty[point[1], point[0]] = 1
            print(f"Frames: {1/(time()-ts)}")
    except ValueError:
        pass

    # Update the display
    pygame.display.flip()

    if not auto:
        pose[0], pose[1], pose[2] = mouse_position[0], mouse_position[1], pose[2]+1
    maptable.putRaw("data", bytearray(np.fliplr((empty+1)).reshape(flatres).tolist()))
    copy_pose = (pose*10)-175
    copy_pose[2] = ((copy_pose[2] + 175)/10)%360
    botTable.putNumberArray("pose", copy_pose)

    # Cap the frame rate
    clock.tick(FPS)
    #print(mouse_position)
    #print(pose)

pygame.quit()
