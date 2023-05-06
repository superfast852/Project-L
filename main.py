from Robots.L import Robot, NetworkTables, sim
from utils.maps.SLAM import SLAM
from extensions.collision import check_collision, check_surroundings

# SLAM Settings
map = "classroom.pkl"
map_m = None
save_map = 1
map_name = None
update_map = 1
draw = 1

# Self-Nav Settings
collision_check_range = (180, 271)
collision_threshold = 20
auto_speed = 0.5

# Initialize robot and algorithms
robot = Robot()  # Automatically sets up the server.
slam = SLAM(robot.lidar, map, map_m, update_map, draw)

# Get network tables for data
joy = NetworkTables.getTable("Joysticks")
stat = NetworkTables.getTable("Status")
data = NetworkTables.getTable("BotData")


def teleop():
    reads = joy.getNumberArray("read", [0, 0, 0, 0, 0])
    driveMode = stat.getBoolean("driveMode", 1)
    robot.drive.mecanum = driveMode
    robot.drive.drive(reads[0], reads[1], reads[4], reads[2])


def self_nav(area):
    robot.drive.mecanum = 1
    if check_collision(area, collision_check_range, collision_threshold):
        side = check_surroundings(area, collision_threshold)
        if side == "Left":
            robot.drive.drive(-1, 1, auto_speed, 0)
        else:
            robot.drive.drive(1, 1, auto_speed, 0)
    else:
        robot.drive.drive(0, 1, auto_speed, 0)


try:
    while not stat.getBoolean("exit", False):

        # Collect data useful for the robot
        scan = robot.lidar.read()
        pose = slam.update(scan)

        # Doing stuff with the robot
        mode = stat.getBoolean("mode", 0)
        if mode == 2:  # Teleoperation
            if not sim:
                teleop()
            # robot.teleop(joy)
        elif mode == 5:  # Autonomous
            self_nav(scan)
            pass

        # Post data necessary
        data.putNumberArray("speeds", [robot.drive.lf, robot.drive.rf, robot.drive.lb, robot.drive.rb])  # Avoid Arrays!
        data.putNumberArray("pose", pose)
        data.putRaw("map", slam.map)
except KeyboardInterrupt:
    pass

if save_map:
    slam.save()

robot.exit()
