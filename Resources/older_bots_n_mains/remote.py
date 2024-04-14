import time

from extensions.tools import XboxController, launchSmartDashboard
from networktables import NetworkTables
from extensions.NavStack import SLAM, RRT, Map
from breezyslam.sensors import RPLidarA1
from roboviz import MapVisualizer

NetworkTables.initialize(server="soji.local")
NetworkTables.setUpdateRate(0.01)
joyTable = NetworkTables.getTable("Joysticks")
status = NetworkTables.getTable("Status")
bot = NetworkTables.getTable("BotData")
dash = NetworkTables.getTable("SmartDashboard")

map = Map('random')
slam = SLAM(RPLidarA1(), map, 35)
planner = RRT(map)
visualizer = MapVisualizer(800, 35, "SLAM", True)


joy = XboxController()
launchSmartDashboard()


mode = 0
drive_mode = 0
min_samples = 50
pose = (400, 400, 0)
rrtCoords = None


def changeMode():
    global mode
    mode = not mode
    status.putBoolean("mode", mode)


def changeDrive():
    global drive_mode
    drive_mode = not drive_mode
    status.putBoolean("driveMode", drive_mode)


joy.setTrigger("RB", changeDrive)
joy.setTrigger("LB", changeMode)

try:
    while True:
        reads = joy.read()
        joyTable.putNumberArray("read", reads[0:5])

        if joy.Start:
            break

        if reads[5]:
            rrtcoords = planner.plan(pose, (pose[0]+50, pose[1]+50))
            if planner.isValidPath(rrtcoords):
                print(rrtcoords)
                status.putNumberArray("path", rrtcoords)
            else:
                print("The path could not be found. Please check your coordinates and try again.")

        distances = list(bot.getNumberArray("distances", [0]*10))
        angles = list(bot.getNumberArray("angles", [0]*10))
        if len(distances) >= min_samples and len(distances) == len(angles):
            pose = slam.update(distances, angles)
        try:
            map_pose = slam.px2pose(pose)
            map_pose = [i/1000 for i in map_pose]
            #visualizer.display(map_pose[0], map_pose[1], map_pose[2], map.toSlam())
        except Exception as e:
            print(f"[ERROR] Map: {e}")

        status.putBoolean("exit", False)
except KeyboardInterrupt:
    pass

print("Exiting...")
for i in range(10):
    status.putBoolean("exit", True)
    time.sleep(0.1)
NetworkTables.stopServer()
NetworkTables.stopClient()
map.save("latest_map")
