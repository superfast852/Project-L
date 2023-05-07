from extensions.tools import XboxController, launchSmartDashboard
from networktables import NetworkTables

NetworkTables.initialize(server="soji.local")

joyTable = NetworkTables.getTable("Joysticks")
status = NetworkTables.getTable("Status")
bot = NetworkTables.getTable("BotData")

dash = NetworkTables.getTable("SmartDashboard")

joy = XboxController()
launchSmartDashboard()

exit_state = 0
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
    exit_sig, exit_state = joy.edge(joy.Start, exit_state)
    mode_change, mode_state = joy.edge(joy.LB, mode_state)
    change_drive, drive_state = joy.edge(joy.RB, drive_state)

    reads = joy.read()
    joyTable.putNumberArray("read", reads)
    send2Dash(reads)

    if exit_sig:
        status.putBoolean("exit", 1)
        break
    if mode_change:
        mode = not mode
        status.putBoolean("mode", mode)
    if change_drive:
        drive_mode = not drive_mode
        status.putBoolean("driveMode", drive_mode)

    if mode:  # Manual
        pass
    else:  # Auto
        pass

print("Exiting...")
NetworkTables.stopClient()
