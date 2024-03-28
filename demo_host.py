from networktables import NetworkTables
from extensions.tools import XboxController
from time import sleep
from _pickle import dumps

NetworkTables.initialize(server='orinnano.local')
controls = NetworkTables.getTable('input')
controller = XboxController()

running = True
while running:
    values = controller.read()
    controls.putRaw("vals", dumps(values))
    sleep(1/30)