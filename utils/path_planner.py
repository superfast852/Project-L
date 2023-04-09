import pygame
import pygame_gui
from pygame import Surface, draw, sprite
from controller import XboxController
from utils import smoothSpeed
import math
from ast import literal_eval

# TODO: Add charging station, Add collision, Add A*, general fixes


w, h = 1920, 930  # Map res
speed = 5

black = (0, 0, 0)
white = (255, 255, 255)
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)
yellow = (255, 255, 0)
cyan = (0, 255, 255)
magenta = (255, 0, 255)

marker_maker = 1
edit_marker = 0
red_positions = ((1676, 868, 0), (1676, 743, 0), (1676, 677, 0), (1676, 545, 0), (1676, 480, 0), (1676, 350, 0))
blue_positions = ((241, 351, 180), (241, 479, 180), (241, 546, 180), (241, 675, 180), (241, 739, 180), (241, 869, 180))
team = red
position = 1

try:
    if team == blue:
        folder = "blue"
    else:
        folder = "red"
    with open(f"{folder}Paths/{team}{position}.txt", "r") as f:
        data = literal_eval(f.read())
        print("Settings: ", data)
        markers = data
        if edit_marker:
            marker_maker = 1
except FileNotFoundError:
    print("No file found")
    if team == blue:
        markers = [blue_positions[position] if position else (w//2, h//2, 0)]
    else:
        markers = [red_positions[position] if position else (w // 2, h // 2, 0)]


class Robot(sprite.Sprite):
    def __init__(self, team, start_pos=0, size=(150, 136)):
        super().__init__()
        if start_pos is None or start_pos > 5:
            spawn = (w // 2, h // 2, 90)
        elif team == red:
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
        self.rotate(spawn[2])

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

    def rotateTo(self, angle, mod_speed=5):  # Rename to rotateTo
        # Calculate the shortest angle between the current and target angles
        angle_diff = ((angle - self.rot) + 180) % 360 - 180
        # Call the original smoothSpeed function with the shortest angle as the target
        if round(angle_diff) != 0:
            rSpeed = smoothSpeed(0, angle_diff, speed_lim=mod_speed, min_speed=0.5)
            self.rotate(rSpeed)
            return rSpeed

    def update(self):
        if self.rect.x < 0:
            self.rect.x = 0
        if self.rect.x+self.rect.size[0] > w:
            self.rect.x = w - self.rect.size[0]

        if self.rect.y < 0:
            self.rect.y = 0
        if self.rect.y + self.rect.size[1] > h:
            self.rect.y = h-self.rect.size[1]

    def collide(self, x, y):
        """Make a function that correctly handles colliding with other sprites.
        It'll only be called when a collision has been detected, and it will pass the point of collision."""


class ChargingStation(sprite.Sprite):
    def __init__(self, side, size=(150, 280)):
        super().__init__()
        self.size = [i // 1 for i in size]
        self.image = Surface(self.size, pygame.SRCALPHA)
        self.image.fill(side)
        self.rect = self.image.get_rect()
        if side == blue:
            self.rect.x, self.rect.y = (375, 473)
        elif side == red:
            self.rect.x, self.rect.y = (1394, 469)
        else:
            self.rect.x, self.rect.y = (0, 0)


def compare_pos(a, b, tol=1):
    if len(a) != len(b):
        return False
    delta = [0,0,0]
    for i in range(len(a)):
        if i == 2:  # treat third value as angle
            angle_diff = abs((a[i] - b[i] + 360 / 2) % 360 - 360 / 2)
            if angle_diff > tol:
                delta[2] = angle_diff
        else:  # treat other values as linear distances
            if abs(a[i] - b[i]) > tol:
                delta[i] = abs(a[i] - b[i])
    print(delta)
    return True if delta == [0, 0, 0] else False


def follow_path(bot, points):
    global current_marker
    if current_marker < len(points):
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


def handle_station_collision(bot, station, px=5):
    if bot.rect.top >= station.rect.top - px and bot.rect.bottom <= station.rect.bottom + px:  # Bot fully inside station
        if bot.rect.bottom > station.rect.bottom:
            bot.rect.bottom = station.rect.bottom
        elif bot.rect.top < station.rect.top:
            bot.rect.top = station.rect.top

    elif bot.rect.top < station.rect.top < bot.rect.bottom - px or bot.rect.top + px < station.rect.bottom < bot.rect.bottom: # Bot on sides
        if not bot.rect.right - px > station.rect.left:
            bot.rect.right = station.rect.left
        elif not bot.rect.left + px < station.rect.right:
            bot.rect.left = station.rect.right

    elif bot.rect.right-px > station.rect.left and bot.rect.left < station.rect.right: # Bot on tops
        if station.rect.bottom > bot.rect.bottom > station.rect.top - px:
            bot.rect.bottom = station.rect.top
        elif bot.rect.top < station.rect.bottom + px:
            bot.rect.top = station.rect.bottom


pygame.init()
screen = pygame.display.set_mode((w, h))
pygame.display.set_caption("Mecanum Drive Test")
clock = pygame.time.Clock()
bots = pygame.sprite.Group()
stations = pygame.sprite.Group()
blue_details = compare_pos((375, 473, 0), (525, 750, 0))
red_details = compare_pos((1394, 469, 0), (1544, 753, 0))
print(red_details, blue_details)

ui = pygame_gui.UIManager((w, h))
hello = pygame_gui.elements.UIButton(pygame.Rect(0, 0, 20, 20), "hi", manager=ui)

bg = pygame.image.load("field.png")
red_charging = ChargingStation(red)
blue_charging = ChargingStation(blue)
stations.add(blue_charging)
stations.add(red_charging)

joysticks = XboxController()

sim_bot = Robot(team, position)
bots.add(sim_bot)

current_marker = 0
done = False
autonomous = False
latches = {"lClick": 0, "rClick": 0, "mClick": 0,
           "a": 0, "b": 0, "x": 0, "y": 0,
           "lBumper": 0, "rBumper": 0, "lStick": 0, "rStick": 0,
           "back": 0, "start": 0}

while not done:
    # Event handling
    time_delta = clock.tick(60) / 1000.0
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE):
            done = True
        if event.type == pygame_gui.UI_BUTTON_PRESSED:
            if event.ui_element == hello:
                print(f"Current markers list: {markers}")
        ui.process_events(event)
    screen.blit(bg, (0, 0))  # Draw background
    ui.update(time_delta)
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

        sim_bot.moveX(joy_in[0] * joy_in[4] * speed)
        sim_bot.moveY(-joy_in[1] * joy_in[4] * speed)
        sim_bot.rotate(-joy_in[2] * speed//2)

        if joysticks.UD:
            sim_bot.rotateTo(90)
        elif joysticks.RD:
            sim_bot.rotateTo(0)
        elif joysticks.LD:
            sim_bot.rotateTo(180)
        elif joysticks.DD:
            sim_bot.rotateTo(270)

        if joy_in[7]:
            # noinspection PyTypeChecker
            bots.remove(sim_bot)
            del sim_bot
            sim_bot = Robot(team, position)
            # noinspection PyTypeChecker
            bots.add(sim_bot)

        if joy_in[5]:
            autonomous = True

        draw_marker_links()
        if marker_maker:
            if latches['rClick'] and lClick:
                mouse_pos = pygame.mouse.get_pos()
                markers.append(mouse_pos + (0,))
            if b:
                markers.append(sim_bot.rect.center + (round(sim_bot.rot),))
            if x:
                markers = []
                if team == "red":
                    markers.append(red_positions[position])
                else:
                    markers.append(blue_positions[position])
            if y:
                markers.pop(-1)
    else:  # Autonomous
        if follow_path(sim_bot, markers):
            print("Path Complete")
            autonomous = False

    #Station Collision
    station_collision = pygame.sprite.groupcollide(bots, stations, False, False)
    colliding_bots = list(station_collision.keys())
    colliding_stations = list(station_collision.values())
    try:
        for i in range(len(colliding_bots)):
            handle_station_collision(colliding_bots[i], colliding_stations[0][i])
    except ValueError:
        pass


    # Rendering
    draw_markers()

    bots.update()
    stations.update()
    bots.draw(screen)
    stations.draw(screen)
    ui.draw_ui(screen)
    pygame.display.flip()
    # clock.tick(60)

print("Captured markers: ", markers)
if marker_maker:
    with open(f"{team}{position}.txt", "w") as f:
        data = str(markers)
        print("Saving data: ", data)
        f.write(data)
pygame.quit()
