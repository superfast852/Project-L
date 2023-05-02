from networktables import NetworkTables
from numpy import random

NetworkTables.initialize()
joy = NetworkTables.getTable("Joysticks")
stat = NetworkTables.getTable("Status")
data = NetworkTables.getTable("BotData")
dash = NetworkTables.getTable("SmartDashboard")
mecanum = NetworkTables.getTable("Mecanum Drivebase")

while True:
    sig = stat.getBoolean("exit", False)
    mode = stat.getBoolean("mode", 0)
    dash.putNumber("lf", random.randn(1)[0])
    dash.putNumber("rf", random.randn(1))
    dash.putNumber("lb", random.randn(1))
    dash.putNumber("rb", random.randn(1))
