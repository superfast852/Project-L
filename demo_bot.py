from networktables import NetworkTables
from Robots.RM_HAL import Drive
from _pickle import loads, dumps
default = dumps([0, 0, 0, 0, 0, 0, 0, 0])

NetworkTables.initialize()
inputs = NetworkTables.getTable('input')
drive = Drive()

running = True
prev_value = 0
state = False
while running:
    vals = loads(inputs.getRaw("vals", default))
    state = 1 if (vals[7] != prev_value and not prev_value) else 0
    prev_value = vals[7]
    if state:
        drive.switchDrive()
    print(drive.drive(vals[0], vals[1], vals[4], 0))
    print(list(map(lambda x: round(x, 2), drive.get_directions())))
