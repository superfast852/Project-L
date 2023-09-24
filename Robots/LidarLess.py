from .HAL import Drive, io as GPIO, time
from networktables import NetworkTables
from Rosmaster_Lib import Rosmaster


class LDrive(Drive):  # Override Communications

    def comms(self, com, baud, update_freq=10):
        self.board = Rosmaster(com=com)
        while self.thread_life:
            time.sleep(1 / update_freq)


class Robot:
    def __init__(self):
        NetworkTables.initialize()
        self.drive = LDrive()
        self.io = GPIO

    def exit(self):
        self.drive.exit()
        self.io.cleanup()
