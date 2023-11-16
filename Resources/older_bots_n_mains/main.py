from Resources.older_bots_n_mains.Loyola import Robot, NetworkTables

publish_lidar = 1

# Self-Nav Settings
collision_check_range = (180, 271)
collision_threshold = 20
auto_speed = 0.5

# Initialize robot and algorithms
robot = Robot()  # Automatically sets up the server.

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

        if stat.getBoolean("mode", 0):  # Autonomous
            # NOTE: There could be a serious jam in here, due to driving mode.
            direction = robot.lidar.self_nav(collision_check_range, collision_threshold, (90, 270), (270, 360))
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
        if publish_lidar:
            data.putNumberArray("distances", distances)
            data.putNumberArray("angles", angles)
except KeyboardInterrupt:
    pass


robot.exit()
