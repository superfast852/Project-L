from evdev import InputDevice
from os import path
from threading import Thread
from time import sleep, time
from extensions.logs import logging
logger = logging.getLogger(__name__)


class XboxController(object):
    MAX_TRIG_VAL = 1024
    MAX_JOY_VAL = 32768

    def __init__(self, deadzone=0.1, retries=5, stall=5, atloss=lambda: []):
        self.retries = retries
        self.stall = stall
        self.atloss = atloss

        self.device_file = ""
        self.gamepad = None
        self.conn_t = None
        self.connect()
        self.deadzone = deadzone

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

        self._monitor_thread = Thread(target=self._monitor_controller, daemon=True)
        self._monitor_thread.start()

    def connect(self):
        existing = []
        names = ["Generic X-Box pad", "Xbox Wireless Controller", "Microsoft X-Box One S pad"]
        for i in range(50):
            if path.exists(f"/dev/input/event{i}"):
                existing.append(f"/dev/input/event{i}")
            if path.exists(f"/dev/input/js{i}"):
                existing.append(f"/dev/input/js{i}")
        for device in existing:
            try:
                gamepad = InputDevice(device)
                if gamepad.name in names:
                    self.device_file = device
                    self.conn_t = time()
                    self.gamepad = gamepad
                    break
            except OSError:
                continue
        else:
            logger.error("[XboxController]: No controller found.")
            raise OSError("No controller found")

    def read(self):  # return the buttons/triggers that you care about in this methode
        reads = [self.LJoyX, self.LJoyY, self.RJoyX, self.RJoyY,
                 self.RT, self.A, self.Back, self.Start]

        return [self._clean(i) for i in reads]

    def _clean(self, x):  # Filter out the inputs.
        return round(x, 3) if not self.deadzone > x > -self.deadzone else 0

    def _monitor_controller(self):
        from evdev import ecodes
        try:
            start = self.conn_t
            for event in self.gamepad.read_loop():
                if self.conn_t != start:
                    break
                if not path.exists(self.device_file):
                    logger.warning("Controller has disconnected. Trying to reconnect...")
                    print("Controller has disconnected. Trying to reconnect...")
                    # Indicates that the controller disconnected.
                    for i in range(self.retries):
                        try:
                            self.connect()
                            # Need to work on this, the reassignment of self.gamepad may break this loop.
                            self._monitor_thread = Thread(target=self._monitor_controller, daemon=True)
                            self._monitor_thread.start()
                            break
                        except OSError:
                            sleep(self.stall)
                    else:  # If the max amount of retries is reached
                        logger.error("[XboxController]: Could not reconnect to controller.")
                        raise OSError("[ERROR] Controller: Could not reconnect to controller.")
                    continue


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
                        self.UD = max(0, -event.value)
                        self.DD = max(0, event.value)

                    elif event.code == 16:
                        self.LD = max(0, -event.value)
                        self.RD = max(0, event.value)

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
                    elif event.code == 314:
                        self.Back = event.value
                    elif event.code == 315:
                        self.Start = event.value

        except Exception as e:
            logger.error(f"[XboxController]: {e}")
            logger.warning("Controller has disconnected. Trying to reconnect...")
            print("Controller has disconnected. Trying to reconnect...")
            self.reset()
            self.atloss()
            # Indicates that the controller disconnected.
            for i in range(self.retries):
                try:
                    self.connect()
                    # Need to work on this, the reassignment of self.gamepad may break this loop.
                    self._monitor_thread = Thread(target=self._monitor_controller, daemon=True)
                    self._monitor_thread.start()
                    break
                except OSError:
                    sleep(self.stall)
            else:  # If the max amount of retries is reached
                logger.error("[XboxController]: Could not reconnect to controller.")
                raise OSError("[ERROR] Controller: Could not reconnect to controller.")

    @staticmethod
    def edge(pulse, last, rising=True):
        status = (pulse if rising else not pulse) and pulse != last
        return status, pulse

    def setTrigger(self, button_name, f, rising=True, polling=1/120, **kwargs):
        last = False
        def trigger(**kwargs):
            nonlocal last
            while True:
                button = getattr(self, button_name)
                pulse, last = self.edge(button, last, rising)
                if pulse:
                    f(**kwargs)
                sleep(polling)
        Thread(target=trigger, kwargs=kwargs, daemon=True).start()
        logger.info(f"[XboxController] Trigger set for {button_name} button.")

    def reset(self):
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


if __name__ == "__main__":
    controller = XboxController()

    def toCall(btn, message):
        print(f"Button {btn} was pressed. {message}")

    controller.setTrigger("A", toCall, message="This is a Rising call.", btn="A")
    controller.setTrigger("A", toCall, message="This is a Falling call.", btn="A", rising=False)
    while True:
        print(controller.read())
        sleep(0.5)