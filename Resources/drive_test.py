from Robots.RM_HAL import Drive
from extensions.tools import getAngle
if __name__ == "__main__":
    from extensions.XboxController import XboxController
    drive = Drive()
    joy = XboxController(0.15)
    joy.setTrigger("RB", drive.switchDrive)
    while True:
        reads = joy.read()
        print(["%.3f" % i for i in drive.drive(reads[0], reads[1], reads[4], reads[2])], getAngle(reads[0], reads[1]))

