from extensions.HAL import Drive
from extensions.tools import getAngle
if __name__ == "__main__":
    from extensions.tools import XboxController
    drive = Drive()
    joy = XboxController(0.15)
    states = {"RB": 0}
    while True:
        reads = joy.read()
        edge, states["RB"] = joy.edge(joy.RB, states['RB'])
        drive.switchDrive() if edge else print(["%.3f" % i for i in drive.drive(reads[0], reads[1], reads[4], reads[2])], getAngle(reads[0], reads[1]))

