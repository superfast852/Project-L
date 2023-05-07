from Robots.L import Robot, NetworkTables
from extensions.NavStack import AutoNav

collision_check_range = (180, 271)
collision_threshold = 20
auto_speed = 0.5
minimum_samples = 100

robot = Robot()
navigator = AutoNav(robot.lidar, 800, 50, 1000)
joy = NetworkTables.getTable("Joysticks")
stat = NetworkTables.getTable("Status")
data = NetworkTables.getTable("BotData")


def teleop():
    reads = joy.getNumberArray("read", [0, 0, 0, 0, 0])
    driveMode = stat.getBoolean("driveMode", 1)
    robot.drive.mecanum = driveMode
    robot.drive.drive(reads[0], reads[1], reads[4], reads[2])


while not stat.getBoolean("exit", False):
    angles, distances = robot.lidar.read()
    if len(angles) > minimum_samples:
        pose = navigator.update(distances, angles=angles)

    if stat.getBoolean("mode", 0):
        direction = navigator.Self_Nav(robot.lidar.map, collision_check_range, collision_threshold)
    else:
        teleop()
