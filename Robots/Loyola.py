from networktables import NetworkTables
from Robots.Loyola_HAL import Drive, RP_A1, GPIO

class Robot:

    def __init__(self):
        NetworkTables.initialize()
        self.drive = Drive()
        self.io = GPIO
        self.lidar = RP_A1()  # LD06()

    def exit(self):
        NetworkTables.stopServer()
        self.drive.exit()
        #self.mpu.exit()
        io.cleanup()
        self.lidar.exit()
        print("Robot exited.")