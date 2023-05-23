from Robots.LidarLess import Robot, time

robot = Robot()  # Automatically sets up the server.

drive = robot.drive

drive.lf = 1
time.sleep(1)
drive.lf = -1
time.sleep(1)
drive.lf = 0

drive.rf = 1
time.sleep(1)
drive.rf = -1
time.sleep(1)
drive.rf = 0

drive.lb = 1
time.sleep(1)
drive.lb = -1
time.sleep(1)
drive.lb = 0

drive.rb = 1
time.sleep(1)
drive.rb = -1
time.sleep(1)
drive.rb = 0
