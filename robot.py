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

    def pickUp(self, posA, posB):
        self.drive.moveTo(posA[0], posA[1], 0)
        self.arm.grab()
        self.drive.moveTo(posB[0], posB[1], 0)


    def exit(self):
        self.drive.exit()
        self.mpu.exit()
        io.cleanup()
        print("Robot exited.")


bot = Robot()
sleep(5)
bot.exit()
