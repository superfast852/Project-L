from networktables import NetworkTables
from extensions.tools import XboxController
from _pickle import dumps

NetworkTables.initialize(server='localhost')
controls = NetworkTables.getTable('input')
controller = XboxController()

running = True
last_start = False
while running:
    values = controller.read()
    press, last_start = controller.edge(controller.Start, last_start)
    controls.putRaw("vals", dumps(values))