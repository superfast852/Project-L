import pygame
from pygame import Surface, draw, sprite
from extensions.tools import XboxController
from extensions.tools import smoothSpeed
import pickle
import math

x, y = 1920, 942
black = (0, 0, 0)
white = (255, 255, 255)
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)
yellow = (255, 255, 0)
cyan = (0, 255, 255)
magenta = (255, 0, 255)

marker_maker = 0
edit_marker = 1
try:
    with open("parameters.pkl", "rb") as f:
        data = pickle.load(f).values()
        print("Settings: ", data)
        markers, team, position = tuple(data)
        if edit_marker:
            marker_maker = 1
except FileNotFoundError:
    markers = [(960, 471, 90), (257, 358, 180), (257, 556, 270.0), (448, 561, 0), (708, 561, 0),
               (589, 372, 115), (258, 359, 180), (257, 487, 180), (447, 664, 0)] \
        if not marker_maker else [(x//2, y//2, 90)]


class Robot(sprite.Sprite):
    def __init__(self, team, start_pos=0, size=(150, 136)):
        super().__init__()
        if team == red:
            spawn = red_positions[start_pos]
        elif team == blue:
            spawn = blue_positions[start_pos]
        else:
            spawn = red_positions[start_pos]

        # Initialization
        self.size = [i // 1 for i in size]
        self.initialImage = Surface(self.size, pygame.SRCALPHA)
        self.image = self.initialImage
        self.image.fill(team)

        # Graphical
        draw.rect(self.image, team, [0, 0, self.size[0], self.size[1]])  # Cart
        draw.rect(self.image, black, (20, 20, 20, 20))  # lt wheel
        draw.rect(self.image, black, (20, size[1] - 40, 20, 20))  # rt wheel
        draw.rect(self.image, black, (size[0] - 40, 20, 20, 20))  # lb wheel
        draw.rect(self.image, black, (size[0] - 40, size[1] - 40, 20, 20))  # rb wheel
        draw.rect(self.image, green, (size[0] - 40, size[1] // 2 - 10, 20, 20))  # Front Locator

        # Physics
        self.rect = self.image.get_rect()
        self.rect.x, self.rect.y = spawn[0] - (size[0] / 2), spawn[1] - (size[1] / 2)
        self.rot = 0
        self.rotate(90)

    def moveX(self, x):
        self.rect.x += x
        return self.rect.x

    def moveY(self, y):
        self.rect.y += y
        return self.rect.y

    def posX(self, x):
        self.rect.x = x - 25
        return self.rect.x

    def posY(self, y):
        self.rect.y = y - 25
        return self.rect.y

    def rotate(self, angle):
        self.rot += angle
        if self.rot > 360 or self.rot < -360:
            self.rot = 0
        old_rect = self.rect.copy()
        self.image = pygame.transform.rotate(self.initialImage, self.rot)
        self.rect = self.image.get_rect(center=old_rect.center)

    def goTo(self, target, speed=5):
        xSpeed, ySpeed, rSpeed = 0, 0, 0
        botX, botY = self.rect.center
        botR = self.rot
        if botX != target[0]:
            xSpeed = smoothSpeed(botX, target[0], speed)
            self.moveX(xSpeed)
        if botY != target[1]:
            ySpeed = smoothSpeed(botY, target[1], speed)
            self.moveY(ySpeed)
        if botR != target[2]:
            rSpeed = self.rotateTo(target[2], speed + 1)
        return (botX, botY, botR), (xSpeed, ySpeed, rSpeed)

    def rotateTo(self, angle, speed=5):
        # Calculate the shortest angle between the current and target angles
        angle_diff = ((angle - self.rot) + 180) % 360 - 180
        # Call the original smoothSpeed function with the shortest angle as the target
        if round(angle_diff) != 0:
            rSpeed = smoothSpeed(0, angle_diff, speed_lim=speed, min_speed=0.5)
            self.rotate(rSpeed)
            return rSpeed


def compare_pos(a, b, tol=1):
    if len(a) != len(b):
        return False
    for i in range(len(a)):
        if i == 2:  # treat third value as angle
            angle_diff = abs((a[i] - b[i] + 360 / 2) % 360 - 360 / 2)
            if angle_diff > tol:
                return False
        else:  # treat other values as linear distances
            if abs(a[i] - b[i]) > tol:
                return False
    return True


def follow_path(bot, points):
    global current_marker
    if current_marker < len(points):
        print(points[current_marker], bot.rect.center + (bot.rot,))
        if compare_pos(bot.rect.center + (bot.rot,), points[current_marker]):
            current_marker += 1
        else:
            bot.goTo(points[current_marker])
            pygame.draw.aaline(screen, black, bot.rect.center, points[current_marker][:2], 2)

    else:
        current_marker = 0
        return 1


def draw_markers():
    for i in range(len(markers)):
        draw.circle(screen, green, markers[i][:2], 10)
        draw.circle(screen, black, markers[i][:2], 10, 1)
        draw.circle(screen, red, markers[i][:2], 5)
        draw.circle(screen, black, markers[i][:2], 5, 1)
        draw.circle(screen, blue, markers[i][:2], 2)
        draw.circle(screen, black, markers[i][:2], 2, 1)
        draw.circle(screen, white, markers[i][:2], 1)


def draw_marker_links():
    for i in range(len(markers) - 1):
        draw.line(screen, black, markers[i][:2], markers[i + 1][:2], 2)


def get_click_angle(x, y, rx, ry, theta):
    dx = x - rx
    dy = y - ry
    angle = math.degrees(math.atan2(dy, dx)) % 360
    dtheta = angle - theta
    if dtheta > 180:
        dtheta -= 360
    elif dtheta < -180:
        dtheta += 360
    return dtheta


pygame.init()
screen = pygame.display.set_mode((x, y))
pygame.display.set_caption("Mecanum Drive Test")
clock = pygame.time.Clock()
sprite_list = pygame.sprite.Group()
bg = pygame.image.load("field.png")

joysticks = XboxController()

red_positions = ((0, 0), (x, y), (x / 2, y / 2))
blue_positions = ((0, 0), (x, y), (x / 2, y / 2))
team = red
position = 2

sim_bot = Robot(team, 2)
sprite_list.add(sim_bot)

current_marker = 0
done = False
autonomous = False
latches = {"lClick": 0, "rClick": 0, "mClick": 0,
           "a": 0, "b": 0, "x": 0, "y": 0,
           "lBumper": 0, "rBumper": 0, "lStick": 0, "rStick": 0,
           "back": 0, "start": 0}

while not done:
    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE):
            done = True
    screen.blit(bg, (0, 0))  # Draw background

    # Input handling
    mouse_in = pygame.mouse.get_pressed(3)
    joy_in = joysticks.read()


    # Game logic
    if not autonomous:  # Manual Control
        if 1:  # Input latching management
            lClick, latches["lClick"] = joysticks.edge(mouse_in[0], latches["lClick"])
            rClick, latches["rClick"] = joysticks.edge(mouse_in[2], latches["rClick"])
            mClick, latches["mClick"] = joysticks.edge(mouse_in[1], latches["mClick"])

            a, latches["a"] = joysticks.edge(joysticks.A, latches["a"])
            b, latches["b"] = joysticks.edge(joysticks.B, latches["b"])
            x, latches["x"] = joysticks.edge(joysticks.X, latches["x"])
            y, latches["y"] = joysticks.edge(joysticks.Y, latches["y"])

            lBumper, latches["lBumper"] = joysticks.edge(joysticks.LB, latches["lBumper"])
            rBumper, latches["rBumper"] = joysticks.edge(joysticks.RB, latches["rBumper"])

            lStick, latches["lStick"] = joysticks.edge(joysticks.LJoyB, latches["lStick"])
            rStick, latches["rStick"] = joysticks.edge(joysticks.RJoyB, latches["rStick"])

            back, latches["back"] = joysticks.edge(joysticks.Back, latches["back"])
            start, latches["start"] = joysticks.edge(joysticks.Start, latches["start"])

        sim_bot.moveX(joy_in[0] * joy_in[4] * 10)
        sim_bot.moveY(joy_in[1] * joy_in[4] * 10)
        sim_bot.rotate(joy_in[2] * 10)

        if joysticks.UD:
            sim_bot.rotateTo(270)
        elif joysticks.RD:
            sim_bot.rotateTo(0)
        elif joysticks.LD:
            sim_bot.rotateTo(180)
        elif joysticks.DD:
            sim_bot.rotateTo(90)

        if joy_in[7]:
            # noinspection PyTypeChecker
            sprite_list.remove(sim_bot)
            del sim_bot
            sim_bot = Robot(team, position)
            # noinspection PyTypeChecker
            sprite_list.add(sim_bot)

        if joy_in[5]:
            autonomous = True

        draw_marker_links()
        if lClick:
            if marker_maker:
                mouse_pos = pygame.mouse.get_pos()
                markers.append(mouse_pos + (get_click_angle(mouse_pos[0], mouse_pos[1],
                                                            markers[0][0], markers[0][1], markers[0][2]),))
                print(markers)

    else:  # Autonomous
        if follow_path(sim_bot, markers):
            print("Path Complete")
            autonomous = False
    # Rendering

    draw_markers()

    sprite_list.update()
    sprite_list.draw(screen)
    pygame.display.flip()
    clock.tick(60)

with open("parameters.pkl", "wb") as f:
    data = {"markers": markers, "team": team, "position": position}
    print("Saving data: ", data)
    pickle.dump(data, f)
pygame.quit()
# left: 145, 427
# mid: 145, 618
# right: 145, 819
# size = 167
# 0.3952 ipp
# 2.53 ppi
