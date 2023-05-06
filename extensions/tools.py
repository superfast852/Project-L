import math
from os import system

import inputs
from inputs import get_gamepad
from threading import Thread


class XboxController(object):
    MAX_TRIG_VAL = 1024
    MAX_JOY_VAL = 32768

    def __init__(self, deadzone=0.1):

        self.deadzone = deadzone
        self.found = False
        self.LJoyY = 0
        self.LJoyX = 0
        self.RJoyY = 0
        self.RJoyX = 0
        self.LT = 0
        self.RT = 0
        self.LB = 0
        self.RB = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LJoyB = 0
        self.RJoyB = 0
        self.Back = 0
        self.Start = 0
        self.LD = 0
        self.RD = 0
        self.UD = 0
        self.DD = 0

        self._monitor_thread = Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def read(self):  # return the buttons/triggers that you care about in this methode
        reads = [self.LJoyX, self.LJoyY, self.RJoyX, self.RJoyY,
                 self.RT, self.A, self.Back, self.Start]

        return [self._clean(i) for i in reads]

    def _clean(self, x):  # Filter out the inputs.
        return round(x, 3) if not self.deadzone > x > -self.deadzone else 0

    def _monitor_controller(self):
        while True:
            try:
                events = get_gamepad()
                for event in events:
                    if event.code == 'ABS_Y':
                        self.LJoyY = self._clean(-(event.state / XboxController.MAX_JOY_VAL))  # normalize between -1 and 1
                    elif event.code == 'ABS_X':
                        self.LJoyX = self._clean(event.state / XboxController.MAX_JOY_VAL)  # normalize between -1 and 1
                    elif event.code == 'ABS_RY':
                        self.RJoyY = self._clean(-(event.state / XboxController.MAX_JOY_VAL))  # normalize between -1 and 1
                    elif event.code == 'ABS_RX':
                        self.RJoyX = self._clean(event.state / XboxController.MAX_JOY_VAL)  # normalize between -1 and 1
                    elif event.code == 'ABS_Z':
                        self.LT = self._clean(event.state / XboxController.MAX_TRIG_VAL)  # normalize between 0 and 1
                    elif event.code == 'ABS_RZ':
                        self.RT = self._clean(event.state / XboxController.MAX_TRIG_VAL)  # normalize between 0 and 1
                    elif event.code == 'BTN_TL':
                        self.LB = event.state
                    elif event.code == 'BTN_TR':
                        self.RB = event.state
                    elif event.code == 'BTN_SOUTH':
                        self.A = event.state
                    elif event.code == 'BTN_NORTH':
                        self.X = event.state  # previously switched with X
                    elif event.code == 'BTN_WEST':
                        self.Y = event.state  # previously switched with Y
                    elif event.code == 'BTN_EAST':
                        self.B = event.state
                    elif event.code == 'BTN_THUMBL':
                        self.LJoyB = event.state
                    elif event.code == 'BTN_THUMBR':
                        self.RJoyB = event.state
                    elif event.code == 'BTN_SELECT':
                        self.Back = event.state
                    elif event.code == 'BTN_START':
                        self.Start = event.state
                    elif event.code == 'ABS_HAT0Y':
                        self.UD = event.state == -1
                        self.DD = event.state == 1
                    elif event.code == 'ABS_HAT0X':
                        self.LD = event.state == -1
                        self.RD = event.state == 1
            except Exception as e:
                pass
    @staticmethod
    def edge(pulse, last, rising=True):
        status = (pulse if rising else not pulse) and pulse != last
        return status, pulse


def smoothSpeed(current, target, speed_lim=1, min_speed=0.1, smoothing_spread=10):
    distance = current-target
    try:
        direction = -distance/abs(distance)
        speed = min(9, (distance/10)**2/smoothing_spread)
        output = (1+speed)*direction/10*speed_lim
        return output if abs(output) > min_speed else min_speed*direction
    except ZeroDivisionError:
        return 0


def getAngle(x, y):
    # Calculate the angle in radians and rotate it counterclockwise by 90 degrees
    # Angle grows counterclockwise
    if x == 0 and y == 0:
        return 0.0
    angle = math.degrees(math.atan2(y, x) - math.pi / 2)
    if angle < 0:
        angle += 360
    # Return the angle in degrees
    return round(angle)


def inTolerance(a, b, tol=1):
    return abs(a - b) <= tol


def getCoordinates(angle):
    # Convert the angle from degrees to radians
    angle = math.radians(angle)
    # Calculate the x and y coordinates
    x = round(math.cos(angle + math.pi/2), 3)
    y = round(math.sin(angle + math.pi/2), 3)
    # Return the coordinates as a tuple
    return x, y


def launchSmartDashboard(path="./Resources/shuffleboard.jar"):
    system(f"java -jar {path} >/dev/null 2>&1 &")


if __name__ == "__main__":
    angle = list(range(0, 361, 10))
    for i in angle:
        x, y = getCoordinates(i)
        print(i, (x, y), getAngle(x, y))

    joy = XboxController(0.15)
    states = {"RB": 0}
    while True:
        reads = joy.read()
        edge, states["RB"] = joy.edge(joy.RB, states['RB'])
        if edge:
            break
        print(reads[3])
