import datetime
from extensions.XboxController import XboxController
from Robots.RM_HAL import Drive, driver
from time import sleep
from extensions.logs import logging
logger = logging.getLogger(__name__)

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

while True:
    try:
        vals = controller.read()
        drive.drive(vals[0], vals[1], vals[4], vals[2])
        if killsig:
            drive.brake()
            logger.info("Exited gracefully.")
            break
        sleep(1/60)
    except Exception as e:
        logger.error(f"[ERROR] demo_bot: {e}\n{e.stacktrace}")
        drive.brake()
logger.info(f"Ended at {datetime.datetime.now().time()}\n\n")
