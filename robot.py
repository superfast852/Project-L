from HAL import *
from time import sleep
from networktables import NetworkTables

# This is where we define our robot and its functionality. We instantiate each component and then
# define functions that utilize various parts in order to accomplish something.
# For example, let's say our robot has an arm, mecanum drive train and camera. We can define a
# function that goes to a position, picks up an item, and then drives to a different position.

# TODO: Think about ideal ways to mix systems. Implement subsystems. Implement more functionality. NetworkTables.

class Robot:
    def __init__(self):
        NetworkTables.initialize()
        self.mpu = MPU()
        self.drive = Drive(mag=self.mpu)
        self.arm = Arm()
        self.io = io

    def pickUp(self, a, b):
        self.drive.moveTo(a[0], a[1], 0)
        self.arm.grab()
        self.drive.moveTo(b[0], b[1], 0)

    def autonomous(self):
        self.drive.moveTo(0, 0, 0)
        self.arm.grab()
        self.drive.moveTo(0, 0, 0)

    def teleop(self, joy):
        # Fix this. It won't work
        reads = joy.read()
        states = {"RB": 0}  # This. It gets reset every time teleop is called.
        switch_driving, states["RB"] = joy.edge(joy.RB, states['RB'])
        if switch_driving:
            self.drive.switchDrive()
        self.drive.drive(reads[0], reads[1], reads[4], reads[2])

    def exit(self):
        NetworkTables.stopServer()
        self.drive.exit()
        self.mpu.exit()
        io.cleanup()
        print("Robot exited.")
