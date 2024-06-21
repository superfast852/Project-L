import datetime
from time import time
from extensions.XboxController import XboxController
from Robots.RM_HAL import Drive, driver, MecanumKinematics
from time import sleep
from extensions.logs import logging
logger = logging.getLogger(__name__)

drive = Drive()
kine = MecanumKinematics()
while True:
    try:
        controller = XboxController(atloss=drive.brake)
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
start = time()
while True:
    try:
        vals = controller.read()
        drive.drive(vals[0], vals[1], vals[4], vals[2])
        print(kine.pose, driver.enc_speed)
        if killsig:
            drive.brake()
            logger.info("Exited gracefully.")
            break
        sleep(1/60)
    except Exception as e:
        drive.brake()
logger.info(f"Ended at {datetime.datetime.now().time()}\n\n")
