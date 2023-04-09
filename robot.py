from interface import *

class Robot:
    def __init__(self, sim=True):
        self.mpu = MPU(sim=sim)
        self.drive = Drive(mag=self.mpu, sim=sim)

print(dir(Robot()))