from interface import *
from time import sleep


class Robot:
    def __init__(self):
        self.mpu = MPU()
        self.drive = Drive(mag=self.mpu)

    def exit(self):
        self.drive.exit()
        self.mpu.exit()
        io.cleanup()
        print("Robot exited.")


bot = Robot()
sleep(5)
bot.exit()
