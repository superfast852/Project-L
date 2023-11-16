from Resources.older_bots_n_mains.LidarLess import Robot, NetworkTables

robot = Robot()  # Automatically sets up the server.

# Get network tables for data
NetworkTables.setUpdateRate(0.01)
joy = NetworkTables.getTable("Joysticks")
stat = NetworkTables.getTable("Status")
data = NetworkTables.getTable("BotData")

while not stat.getBoolean("exit", False):  # If an exit signal hasn't been sent

    # Collect data useful for the robot
    reads = joy.getNumberArray("read", [0, 0, 0, 0, 0])
    robot.drive.mecanum = stat.getBoolean("driveMode", 1)

    # Do something with the data
    robot.drive.drive(reads[0], reads[1], reads[4], reads[2])

    # Post data necessary
    data.putNumberArray("speeds", [robot.drive.lf, robot.drive.rf, robot.drive.lb, robot.drive.rb])  # Avoid Arrays!

print("Exit Detected!")
robot.exit()
