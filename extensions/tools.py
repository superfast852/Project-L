import math
from os import system
from threading import Thread
from numpy import linspace
from numba import njit


class XboxController(object):
    MAX_TRIG_VAL = 1024
    MAX_JOY_VAL = 32768

    def __init__(self, deadzone=0.1):
        from evdev import InputDevice
        i = 0
        while True:
            try:
                self.gamepad = InputDevice(f'/dev/input/event{i}')
                break
            except OSError:
                i += 1
                if i > 50:
                    raise OSError("No controller found")
        self.deadzone = deadzone
        self.found = False
        self.LJoyY = 0
        self.LJoyX = 0  # This. Also normalize joystick values
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
        from evdev import ecodes
        try:
            for event in self.gamepad.read_loop():
                # Axis
                if event.type == ecodes.EV_ABS:
                    if event.code == 1:
                        self.LJoyY = self._clean(-(event.value / XboxController.MAX_JOY_VAL) + 1)  # normalize between -1 and 1
                    elif event.code == 0:
                        self.LJoyX = self._clean(event.value / XboxController.MAX_JOY_VAL - 1)  # normalize between -1 and 1
                    elif event.code == 5:
                        self.RJoyY = self._clean(-(event.value / XboxController.MAX_JOY_VAL) + 1)  # normalize between -1 and 1
                    elif event.code == 2:
                        self.RJoyX = self._clean(event.value / XboxController.MAX_JOY_VAL) - 1  # normalize between -1 and 1
                    elif event.code == 10:
                        self.LT = self._clean(event.value / XboxController.MAX_TRIG_VAL)  # normalize between 0 and 1
                    elif event.code == 9:
                        self.RT = self._clean(event.value / XboxController.MAX_TRIG_VAL)  # normalize between 0 and 1
                        # DPad

                    elif event.code == 17:
                        if event.value == 1:
                            self.UD = 1
                            self.DD = 0
                        elif event.value == -1:
                            self.UD = 0
                            self.DD = 1

                    elif event.code == 16:
                        if event.value == 1:
                            self.LD = 1
                            self.RD = 0
                        elif event.value == -1:
                            self.LD = 0
                            self.RD = 1

                elif event.type == ecodes.EV_KEY:
                    # Bumpers
                    if event.code == 310:
                        self.LB = event.value
                    elif event.code == 311:
                        self.RB = event.value

                    # Face Buttons
                    elif event.code == 304:
                        self.A = event.value
                    elif event.code == 307:
                        self.X = event.value  # previously switched with X
                    elif event.code == 308:
                        self.Y = event.value  # previously switched with Y
                    elif event.code == 305:
                        self.B = event.value

                    # Joystick Buttons
                    elif event.code == 317:
                        self.LJoyB = event.value
                    elif event.code == 318:
                        self.RJoyB = event.value

                    # Menu Buttons
                    elif event.code == 158:
                        self.Back = event.value
                    elif event.code == 315:
                        self.Start = event.value

        except Exception as e:
            print(e)

    @staticmethod
    def edge(pulse, last, rising=True):
        status = (pulse if rising else not pulse) and pulse != last
        return status, pulse


@njit
def smoothSpeed(x, target, steps=100):
    height = abs(x-target)
    if height == 0:
        return [0.0, 0.0]
    slope = 10/height if target < x else -10/height
    yshift = min(x, target)

    return [(height / (1 + 2.71828**(slope*i+5))+yshift) \
            for i in linspace(0 if x < target else -height, height if x < target else 0, steps)]


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


smoothSpeed(0.0, 0.0)  # Call it to compile the function
if __name__ == "__main__":
    joy = XboxController(0.15)
    states = {"RB": 0}
    while True:
        reads = joy.read()
        edge, states["RB"] = joy.edge(joy.RB, states['RB'])
        if edge:
            break
        print(reads[3])
