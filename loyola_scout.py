from Robots.L import Robot
from networktables import NetworkTables
from utils.maps.SLAM import SLAM
from extensions.collision import check_collision, check_surroundings

collision_check_range = (180, 271)
collision_threshold = 20
auto_speed = 0.5

robot = Robot()
slam = SLAM(robot.lidar, "classroom.pkl", None, 1, 1)
joy = NetworkTables.getTable("Joysticks")
stat = NetworkTables.getTable("Status")
data = NetworkTables.getTable("BotData")


def self_nav(area):
    robot.drive.mecanum = 0
    if check_collision(area, collision_check_range, collision_threshold):
        side = check_surroundings(area, collision_threshold)
        if side == "Left":
            robot.drive.drive(-1, 1, auto_speed, 0)
        else:
            robot.drive.drive(1, 1, auto_speed, 0)
    else:
        robot.drive.drive(0, 1, auto_speed, 0)


def teleop():
    reads = joy.getNumberArray("read", [0, 0, 0, 0, 0])
    driveMode = stat.getBoolean("driveMode", 1)
    robot.drive.mecanum = driveMode
    robot.drive.drive(reads[0], reads[1], reads[4], reads[2])


while not stat.getBoolean("exit", False):
    angles, distances = robot.lidar.read()
    pose = slam.update(distances, angles=angles)

    if stat.getBoolean("mode", 0):
        self_nav(robot.lidar.map)
    else:
        teleop()
