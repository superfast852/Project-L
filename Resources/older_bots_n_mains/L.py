from Robots.RM_HAL import *
from networktables import NetworkTables

# This is where we define our robot and its functionality. We instantiate each component and then
# define functions that utilize various parts in order to accomplish something.
# For example, let's say our robot has an arm, mecanum drive train and camera. We can define a
# function that goes to a position, picks up an item, and then drives to a different position.

# TODO: Think about ideal ways to mix systems. Implement subsystems. Implement more functionality.


class Robot:

    def __init__(self):
        NetworkTables.initialize()
        self.lidar = RP_A1()  # LD06()
        self.drive = Drive()
        self.io = driver
        self.imu = IMU()
        self.arm = Arm()

    def thread(self, *args):
        pass

    def pickUp(self, a, b):
        self.drive.moveTo(a[0], a[1], 0)
        self.arm.grab()
        self.drive.moveTo(b[0], b[1], 0)

    def test(self, drive=0, arm=0, mpu=0):
        if drive:
            self.drive.moveTo(1, 0, 45)
            self.drive.moveTo(1, 1, 90)
            self.drive.moveTo(0, 1, 180)
            self.drive.moveTo(0, 0, 0)
        if arm:
            self.arm.move(self.arm.grabbing)
            self.arm.grab()
            self.arm.move(self.arm.dropping)
            self.arm.drop()
            self.arm.move(self.arm.home)
        if mpu:
            print(self.imu.getMag(), self.imu.getGyro(), self.imu.getAccel())

    def exit(self):
        NetworkTables.stopServer()
        self.drive.exit()
        self.io.cleanup()
        self.imu.exit()
        print("Robot exited.")


class AdvancedBot(Robot):
    def __init__(self, map=800):
        super().__init__()
        from extensions.NavStack import RRT, SLAM, Map
        self.map = Map(map)
        self.rrt = RRT(self.map)
        self.slam = SLAM(self.lidar, self.map)


if __name__ == '__main__':
    robot = Robot(800)
    robot.goTo()
