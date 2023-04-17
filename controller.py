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
                self.found = True
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
            except:
                print("Controller not found.")
                self.found = False

    @staticmethod
    def edge(pulse, last, rising=True):
        status = (pulse if rising else not pulse) and pulse != last
        return status, pulse


if __name__ == '__main__':
    joy = XboxController()
    while True:
        ins = joy.read()
        print(ins)
