from extensions.tools import XboxController, launchSmartDashboard
from networktables import NetworkTables
from extensions.NavStack import SLAM, RRT, Map
from Robots.HAL import RP_A1

NetworkTables.initialize(server="soji.local")
NetworkTables.setUpdateRate(0.01)
joyTable = NetworkTables.getTable("Joysticks")
status = NetworkTables.getTable("Status")
bot = NetworkTables.getTable("BotData")
map = Map("random")
slam = SLAM(RP_A1(), map, 35)
planner = RRT()
min_samples = 50
pose = (0, 0, 0)
rrtcoords = None

dash = NetworkTables.getTable("SmartDashboard")

joy = XboxController()
launchSmartDashboard()

mode_state = 0
drive_state = 0
mode = 0
drive_mode = 0


def send2Dash(ins):
    names = ["lx", "ly", "rx", "ry", "rt", "a", "back", "start"]
    for name, val in zip(names, ins):
        dash.putNumber(name, val)


while True:
    # Edge Triggers
    mode_change, mode_state = joy.edge(joy.LB, mode_state)
    change_drive, drive_state = joy.edge(joy.RB, drive_state)

    reads = joy.read()
    reads[0] = -reads[0]  # REMEMBER THIS.
    joyTable.putNumberArray("read", reads[0:5])

    if joy.Start:
        for i in range(100):
            status.putBoolean("exit", 1)
        NetworkTables.stopServer()
        NetworkTables.stopClient()
        break
    if mode_change:
        mode = not mode
        status.putBoolean("mode", mode)
    if change_drive:
        drive_mode = not drive_mode
        status.putBoolean("driveMode", drive_mode)
    if reads[5]:
        rrtcoords = planner.plan(pose, [i+50 for i in pose], map)

    distances = list(bot.getNumberArray("distances", [0]*10))
    angles = list(bot.getNumberArray("angles", [0]*10))
    if len(distances) >= min_samples and len(distances) == len(angles):
        pose = slam.update(distances, angles)


print("Exiting...")
NetworkTables.stopClient()
