from interface import *
from time import sleep

# This is where we define our robot and its functionality. We instantiate each component and then
# define the robot's actions using all of them. We can then call these actions from our main.py
# For example, let's say our robot has an arm, mecanum drive train and camera. We can define a
# function that goes to a position, picks up an item, and then drives to a different position.

# TODO: Think about ideal ways to mix systems. Implement subsystems.

class Robot:
    def __init__(self):
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
        reads = joy.read()
        states = {"RB": 0}
        switch_driving, states["RB"] = joy.edge(joy.RB, states['RB'])
        if switch_driving:
            self.drive.switchDrive()
        self.drive.drive(reads[0], reads[1], reads[4], reads[2])

    def exit(self):
        self.drive.exit()
        self.mpu.exit()
        io.cleanup()
        print("Robot exited.")


bot = Robot()
sleep(5)
bot.exit()
