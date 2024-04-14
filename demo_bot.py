import sys
import datetime
now = datetime.datetime.now
sys.stdout = open("out.log", "a")
sys.stderr = sys.stdout
print(f"Started at {now()}")

from extensions.tools import XboxController
from Robots.RM_HAL import Drive
from time import sleep

drive = Drive()
while True:
    try:
        controller = XboxController()
        break
    except OSError:
        pass

killsig = False


def kill():
    drive.brake()
    print("Exited gracefully.")
    killsig = True


controller.setTrigger("Back", kill)
controller.setTrigger("Start", drive.switchDrive)

while True:
    try:
        vals = controller.read()
        drive.drive(vals[0], vals[1], vals[4], vals[2])
        if killsig:
            drive.brake()
            print("Exited gracefully.")
            break
        sleep(1/60)
    except Exception as e:
        print(f"[ERROR] demo_bot: {e}\n{e.stacktrace}")
        kill()
print(f"Ended at {now()}\n\n")
sys.stdout.close()
