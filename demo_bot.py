import datetime
from time import time
from matplotlib import pyplot as plt
from extensions.XboxController import XboxController
from Robots.RM_HAL import Drive, driver
from time import sleep
from extensions.logs import logging
logger = logging.getLogger(__name__)


class AnimatedWindow:
    def __init__(self):
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.gca()
        self.figid = id(self.fig)

    def scatter(self, points, c=(255, 0, 0)):
        self.ax.scatter(zip(*points), c=c)

    def refresh(self):
        if id(plt.gcf()) != self.figid:
            raise ValueError("Window does not exist.")

        plt.draw()
        plt.pause(0.0001)

    def clear(self):
        self.ax.cla()


drive = Drive()
while True:
    try:
        controller = XboxController()
        break
    except OSError:
        pass

killsig = False


def kill():
    global killsig
    killsig = True


controller.setTrigger("Back", kill)
controller.setTrigger("Start", drive.switchDrive)
controller.setTrigger("A", drive.brake)
controller.setTrigger("LB", lambda: driver.set_beep(100))
window = AnimatedWindow()
start = time()
while True:
    try:
        dt = time() - start
        vals = controller.read()
        drive.drive(vals[0], vals[1], vals[4], vals[2])
        if killsig:
            drive.brake()
            logger.info("Exited gracefully.")
            break
        speeds = driver.enc_speed.copy()
        window.scatter([(dt, i) for i in speeds])
        window.refresh()
        sleep(1/60)
    except Exception as e:
        drive.brake()
logger.info(f"Ended at {datetime.datetime.now().time()}\n\n")
