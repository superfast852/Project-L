from robot import Robot, NetworkTables, sim

robot = Robot()  # Automatically sets up the server.
joy = NetworkTables.getTable("Joysticks")
stat = NetworkTables.getTable("Status")
data = NetworkTables.getTable("BotData")

while True:
    sig = stat.getBoolean("exit", False)
    mode = stat.getBoolean("mode", 0)
    data.putNumberArray("speeds", [robot.drive.lf, robot.drive.rf, robot.drive.lb, robot.drive.rb])  # Avoid Arrays!
    if mode == 2:  # if mode
        print("Teleop")
        if not sim:
            reads = joy.getNumberArray("read", [0, 0, 0, 0])
            switch_driving = joy.getNumberArray("driveMode", 0)
            if switch_driving:
                robot.drive.switchDrive()
            robot.drive.drive(reads[0], reads[1], reads[4], reads[2])
        #robot.teleop(joy)
    elif mode == 5:  # else
        print("Autonomous")
        # robot.autonomous()