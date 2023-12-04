import pygame
import math
import numpy as np

# Initialize pygame
pygame.init()

# Movement speed
MOVE_SPEED = 1
TURN_SPEED = 0.1  # In radians

# Initial robot position and orientation
robot_x = 0
robot_y = 0
robot_orientation = 0  # Angle in radians


tpr = 362
r = 10
w2c = 21.7


# These are currently local travel coordinates. We need a way to turn that into global position.
# The basic way is to just add sine and cosine to the position, with the angle being yaw.
"""
global_x = 0
global_y = 0
global_w = 0

First get the tick based travel.

local_pose = kine.ik(wheels)

global_w = local_pose[2]
global_x = local_pose[0]*cos(global_w)
global_y = local_pose[1]*sin(global_w)

The issue with this is we don't know the behavior of local_pose with relation to global.
What i mean is:
Action              Local Position      Global Position
Bot goes forward:   1, 0, 0             1, 0, 0
Bot Turns 45 deg:   1, 0, pi/4          1, 0, pi/4
Bot goes forward:   2, 0, pi/4          1+1*cos(pi/4), 0+1*sin(pi/4), pi/4

The idea would be recurrent addition. So, we get the traveled distance, and add to position.

x, y, z = 0, 0, 0

prev_ticks = 0, 0, 0, 0
ticks = driver.encoders
travel = prev_ticks-ticks
prev_ticks = ticks.copy()

tx, ty, tw = kine.fk(travel)
z += tw
x += tx*cos(z)
y += ty*sin(z)


"""
class Kinematics:
    def __init__(self, flipped_axis=0):
        self.fa = flipped_axis
        self.tpr = 362
        self.r = 10
        self.w2c = 21.7
        if flipped_axis:
            self.x = lambda lf, rf, lb, rb: (lf + rf + lb + rb) * 2.5
            self.y = lambda lf, rf, lb, rb: (-lf + rf + lb - rb) * 2.5
            self.get_lf = lambda y, x, z: (x - y - self.w2c * z) / self.r
            self.get_rf = lambda y, x, z: (x + y + self.w2c * z) / self.r
            self.get_lb = lambda y, x, z: (x + y - self.w2c * z) / self.r
            self.get_rb = lambda y, x, z: (x - y + self.w2c * z) / self.r
        else:
            self.x = lambda lf, rf, lb, rb: (lf + rf + lb + rb) * 2.5
            self.y = lambda lf, rf, lb, rb: (-lf + rf + lb - rb) * 2.5
            self.get_lf = lambda x, y, z: (x - y - self.w2c * z) / self.r
            self.get_rf = lambda x, y, z: (x + y + self.w2c * z) / self.r
            self.get_lb = lambda x, y, z: (x + y - self.w2c * z) / self.r
            self.get_rb = lambda x, y, z: (x - y + self.w2c * z) / self.r
        self.w = lambda lf, rf, lb, rb: (-lf + rf - lb + rb) * 0.11520737327188943
        self.fk = lambda lf, rf, lb, rb: (self.x(lf, rf, lb, rb), self.y(lf, rf, lb, rb), self.w(lf, rf, lb, rb))
        self.ik = lambda x, y, z: (self.get_lf(x, y, z), self.get_rf(x, y, z), self.get_lb(x, y, z), self.get_rb(x, y, z))
        self.vec = lambda x, y, z: (math.hypot(x, y), math.atan2(y, x))

    def update_absolute(self, ticks, current):
        travel = 0


def updateBot():
    keys = pygame.key.get_pressed()
    x, y, z = 0, 0, 0
    if keys[pygame.K_w]:
        x = 1
    if keys[pygame.K_s]:
        x = -1  # Negative because pygame's y-axis is inverted
    if keys[pygame.K_a]:
        y = -1
    if keys[pygame.K_d]:
        y = 1
    if keys[pygame.K_RIGHT]:
        z = -1
    if keys[pygame.K_LEFT]:
        z = 1

    return x, y, z


def update_bot_orientation_to_target(robot_x, robot_y, robot_orientation, target):
    # Calculate the desired orientation for the robot to point to the target
    desired_orientation = math.atan2(target[0] - robot_x, target[1] - robot_y)

    # Calculate the angular difference between the robot's current orientation and the desired orientation
    angular_difference = desired_orientation - robot_orientation + math.pi/2

    # Adjust the angular difference for wrap-around at 2*pi
    while angular_difference > math.pi:
        angular_difference -= 2 * math.pi
    while angular_difference < -math.pi:
        angular_difference += 2 * math.pi

    # Determine the turn command based on the angular difference (reversed directions)
    if angular_difference > 0:
        z = -1  # Turn right
    elif angular_difference < 0:
        z = 1  # Turn left
    else:
        z = 0  # Don't turn

    return z


# Create a display screen
screen = pygame.display.set_mode((400, 400))
kine = Kinematics()
ticks = [0, 0, 0, 0]
# Initialize clock for controlling FPS
clock = pygame.time.Clock()
FPS = 10  # Target frames per second
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    x, y, z = updateBot()
    ikvals = kine.ik(x, y, z)
    fkvals = kine.fk(*ikvals)
    print(fkvals, ikvals, (x, y, z))

    # Cap the frame rate
    clock.tick(FPS)

pygame.quit()
