import pygame
import math
import numpy as np
from extensions.tools import getAngle, line2dots
from extensions.NavStack import PathFollow

# Initialize pygame
pygame.init()

# Colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLACK = (0, 0, 0)

# Screen dimensions
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

# Robot dimensions
ROBOT_RADIUS = 30

# Movement speed
MOVE_SPEED = 1
TURN_SPEED = 0.1  # In radians

# Initial robot position and orientation
robot_x = SCREEN_WIDTH // 2
robot_y = SCREEN_HEIGHT // 2
robot_orientation = 0  # Angle in radians


class Drive:  # TODO: Implement self.max properly
    max_speed = 1.0  # Measure this later.

    def __init__(self, max_speed=1):
        self.lf = 0
        self.rf = 0
        self.lb = 0
        self.rb = 0
        self.max = max_speed
        self.mecanum = 1
        self.x = lambda lf, rf, lb, rb: (lf - rf - lb + rb) * 0.25
        self.y = lambda lf, rf, lb, rb: (lf + rf + lb + rb) * 0.25
        self.w = lambda lf, rf, lb, rb: (lf - rf + lb - rb) * 0.25
        self.test_speeds = ((1, 1, 1, 1), (-1, -1, -1, -1),  # Forward, Backward
                            (-1, 1, 1, -1), (1, -1, -1, 1),  # Left, Right
                            (0, 1, 1, 0), (-1, 0, 0, -1),    # TopLeft, BottomLeft
                            (1, 0, 0, 1), (0, -1, -1, 0),    # TopRight, BottomRight
                            (1, -1, 1, -1), (-1, 1, -1, 1))  # RotateRight, RotateLeft

    # Movement Functions
    def cartesian(self, x, y, speed=1, turn=0):  # WIth standard movement axis
        if not x and not y:  # Sim only.
            self.lf, self.lb, self.rf, self.rb = 0, 0, 0, 0
            return 0, 0, 0, 0
        rawTheta = getAngle(x, -y)
        # TODO: this fitting is broken. Please fix.
        #theta = rawTheta - (math.pi / 2) if rawTheta > (math.pi / 2) else (2 * math.pi) - rawTheta
        #print(math.degrees(rawTheta), theta)
        theta = rawTheta

        sin = math.sin(theta+math.pi/4)
        cos = math.cos(theta+math.pi/4)
        lim = max(abs(sin), abs(cos))

        lf = speed * cos / lim + turn
        rf = speed * sin / lim - turn
        lb = speed * sin / lim + turn
        rb = speed * cos / lim - turn

        clip = speed+abs(turn)

        if clip > 1:
            lf /= clip
            rf /= clip
            lb /= clip
            rb /= clip

        self.lf = lf
        self.rf = rf
        self.lb = lb
        self.rb = rb
        self.comms()
        return lf, rf, lb, rb

    def tank(self, V, omega):
        """
        Convert differential drive commands to mecanum wheel speeds.

        Parameters:
        - V: Forward velocity (-1 to 1).
        - omega: Rotational velocity (-1 to 1), positive is clockwise.
        - width: Distance between left and right wheels. This is a scaling factor and doesn't need to be the actual width of the robot.

        Returns:
        Tuple containing normalized wheel speeds for (FL, FR, BL, BR).
        """

        V_FL_BL = V - omega / 2
        V_FR_BR = V + omega / 2

        # Find the maximum absolute value among all speeds
        max_speed = max(abs(V_FL_BL), abs(V_FR_BR))

        # If the max speed is greater than 1, we need to scale down all speeds
        if max_speed > 1:
            V_FL_BL /= max_speed
            V_FR_BR /= max_speed
        self.lf, self.rf, self.lb, self.rb = V_FL_BL, V_FR_BR, V_FL_BL, V_FR_BR
        self.comms()
        return V_FL_BL, V_FR_BR, V_FL_BL, V_FR_BR


    # Note: For simulation, override this class to calculate the robot's position based on the motors. Also ticks.
    def comms(self, update_freq=10):
        global robot_x, robot_y, robot_orientation
        x = self.x(self.lf, self.rf, self.lb, self.rb)
        y = self.y(self.lf, self.rf, self.lb, self.rb)
        z = self.w(self.lf, self.rf, self.lb, self.rb)
        print(f"Coords: {x, y, z} | Speeds: {[self.lf, self.rf, self.lb, self.rb]}")
        robot_x += x*MOVE_SPEED
        robot_y -= y*MOVE_SPEED
        robot_orientation += z*TURN_SPEED


    def drive(self, x, y, power, turn):
        if self.mecanum:
            return self.cartesian(x, y, power, turn)
        else:
            return self.tank(power, turn)

    def switchDrive(self):
        self.mecanum = not self.mecanum


def updateBot():
    keys = pygame.key.get_pressed()
    x, y, z = 0, 0, 0
    if keys[pygame.K_w]:
        y = 1
    if keys[pygame.K_s]:
        y = -1  # Negative because pygame's y-axis is inverted
    if keys[pygame.K_a]:
        x = -1
    if keys[pygame.K_d]:
        x = 1
    if keys[pygame.K_RIGHT]:
        z = -1
    if keys[pygame.K_LEFT]:
        z = 1

    return x, y, z


def ecd(a, b):
    return np.linalg.norm(np.array(a)-np.array(b))


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


def find_goal_point(path, robot_position, look_ahead_distance):
    closest_point = None
    min_distance = float('inf')

    for segment in path:
        for point in segment:  # Iterate through the sub-points of each segment
            # Check distance to each point on the path
            dist = ecd(robot_position, point)  # Euclidean distance.

            # Update closest point if this point is closer
            if dist < min_distance:
                min_distance = dist
                closest_point = point

            # Return the point if it's farther than the look-ahead distance
            # if dist > look_ahead_distance:
            #     return point

    # If no point is found that's farther than the look-ahead distance, return the closest point
    return closest_point


def pure_pursuit(path, bot_pose, look_ahead_distance):
    robot_position = bot_pose[:2]
    goal_point = find_goal_point(path, robot_position, look_ahead_distance)
    print(goal_point)
    return update_bot_orientation_to_target(*bot_pose, goal_point)


def rudimentary_movement(x, y, z):
    global robot_x, robot_y, robot_orientation
    robot_x += x*MOVE_SPEED
    robot_y -= y*MOVE_SPEED
    robot_orientation += z*TURN_SPEED


# Create a display screen
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption('Robot Simulation')
drive = Drive()

# Initialize clock for controlling FPS
clock = pygame.time.Clock()
FPS = 60  # Target frames per second
path = [[[400, 300], [0, 0]], [[0, 0], [0, 600]], [[0, 600], [800, 600]]]
dotted = [line2dots(line) for line in path]
look_ahead_distance = 40
follower = PathFollow(path)
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Clear the screen
    screen.fill(WHITE)
    drive.comms()

    y, x, z = updateBot()
    za = 0
    #z = pure_pursuit(path, (robot_x, robot_y, robot_orientation), look_ahead_distance)
    #print(follower((robot_x, robot_y, robot_orientation)))
    #print(x, y, z)

    # Circle\
    dots = [(robot_x+look_ahead_distance*math.cos(i), robot_y+look_ahead_distance*math.sin(i)) for i in np.linspace(0, np.pi*2, 720)]
    for dot in dots:
        pygame.draw.circle(screen, BLACK, dot, 1)


    # do not touch :)
    #drive.cartesian(x, y, turn=z if za == 0 else za)
    drive.tank(x,z)
    #rrudimentary_movement(x, y, z)
    # Look Ahead Circle
    pygame.draw.circle(screen, BLACK, (int(robot_x), int(robot_y)), look_ahead_distance,  1)
    # Robot
    pygame.draw.circle(screen, RED, (int(robot_x), int(robot_y)), ROBOT_RADIUS)
    # Target
    # pygame.draw.line(screen, (0, 255, 0), (int(robot_x), int(robot_y)), line[1])
    pygame.draw.line(screen, (255, 0, 0), (int(robot_x), int(robot_y)), (0, int(robot_y)))
    pygame.draw.line(screen, (255, 0, 0), (int(robot_x), int(robot_y)), (int(robot_x), 0))
    # Draw a line to indicate the robot's orientation
    line_end_x = robot_x + ROBOT_RADIUS * math.cos(robot_orientation)
    line_end_y = robot_y - ROBOT_RADIUS * math.sin(robot_orientation)  # Negative because pygame's y-axis is inverted
    pygame.draw.line(screen, BLACK, (robot_x, robot_y), (line_end_x, line_end_y), 3)

    # Update the display
    pygame.display.flip()

    # Cap the frame rate
    clock.tick(FPS)

pygame.quit()
