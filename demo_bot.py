import datetime
from time import time
from extensions.XboxController import XboxController
from Robots.RM_HAL import Drive, driver, MecanumKinematics
from time import sleep
from extensions.logs import logging
logger = logging.getLogger(__name__)
vbat = -1.0
while vbat < 0:
    vbat = driver.get_battery_voltage()
print(f"Battery level: {vbat} V")

drive = Drive()
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
        print([round(i, 3) for i in driver.enc_speed])
        drive.get_directions()
        if killsig:
            drive.brake()
            logger.info("Exited gracefully.")
            break
        sleep(1/60)
    except Exception as e:
        drive.brake()
        print(e)
        break
logger.info(f"Ended at {datetime.datetime.now().time()}\n\n")
