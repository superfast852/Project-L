from Robots.LidarLess import Robot, time

robot = Robot()  # Automatically sets up the server.

drive = robot.drive

directions = ((0, 1), (0, -1), (-1, 0), (1, 0), (1, 1), (-1, -1), (-1, 1), (1, -1))
for direction in directions:
    speeds = drive.cartesian(direction[0], direction[1], speed=0.25)
    names = ("Lf", "Rf", "Lb", "Rb")
    time.sleep(2)
robot.exit()
