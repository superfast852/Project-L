from Robots.Loyola import Robot, NetworkTables
from extensions.NavStack import RRT, SLAM, Map

# SLAM Settings
map = "random"
map_m = None
save_map = 1
map_name = None
update_map = 1
publish_lidar = 1
publish_map = 1

# Self-Nav Settings
collision_check_range = (180, 271)
collision_threshold = 20
auto_speed = 0.5

# Initialize robot and algorithms
robot = Robot()  # Automatically sets up the server.
map = Map(map)
planner = RRT()
slam = SLAM(robot.lidar, map, map_m, update_map)

# Get network tables for data
NetworkTables.setUpdateRate(0.01)
joy = NetworkTables.getTable("Joysticks")
stat = NetworkTables.getTable("Status")
data = NetworkTables.getTable("BotData")
pose = (0, 0, 0)


def teleop():
    reads = joy.getNumberArray("read", [0, 0, 0, 0, 0])
    driveMode = stat.getBoolean("driveMode", 1)
    robot.drive.mecanum = driveMode
    robot.drive.drive(reads[0], reads[1], reads[4], reads[2])


try:
    while not stat.getBoolean("exit", False):  # If an exit signal hasn't been sent

        # Collect data useful for the robot
        angles, distances = robot.lidar.read()
        if (len(angles) > 100) and (len(distances) == len(angles)):
            pose = slam.update(distances, angles)

        if stat.getBoolean("mode", 0):  # Autonomous
            direction = robot.lidar.self_nav(collision_check_range, collision_threshold)
            if direction == "Forward":
                robot.drive.drive(0, 1, auto_speed, 0)
            elif direction == "Left":
                robot.drive.drive(1, 0, auto_speed, 0)
            elif direction == "Right":
                robot.drive.drive(-1, 0, auto_speed, 0)
        else:  # Teleoperation
            teleop()

        # Post data necessary
        data.putNumberArray("speeds", [robot.drive.lf, robot.drive.rf, robot.drive.lb, robot.drive.rb])  # Avoid Arrays!
        data.putNumberArray("pose", pose)
        if publish_map:
            data.putRaw("map", map.map)
        if publish_lidar:
            data.putNumberArray("distances", distances)
            data.putNumberArray("angles", angles)
except KeyboardInterrupt:
    pass

if save_map:
    map.save(map_name)

robot.exit()
