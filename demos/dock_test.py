from extensions.NavStack import Map
from networktables import NetworkTables
from threading import Thread
from time import sleep

map = Map("random")
pose = (400, 400, 0)
NetworkTables.initialize()
map_table = NetworkTables.getTable("map")

def update():
    global map_table, map
    while True:
        map_table.putRaw("map", bytes(map.toSlam()))
        map_table.putNumberArray("pose", pose)
        sleep(1)

thread = Thread(target=update, daemon=True)
thread.start()
while True:
    map.animate()
    sleep(1/60)