from networktables import NetworkTables
from extensions.NavStack import Map
from threading import Thread
from time import sleep

map = Map(800)
pose = (400, 400, 0)
NetworkTables.initialize("orinnano.local")
map_table = NetworkTables.getTable("map")


def update():
    global map_table, map, pose
    while True:
        mapbytes = map_table.getRaw("map", bytearray([127]*800*800))
        pose = map_table.getNumberArray("pose", (400, 400, 0))
        map.fromSlam(mapbytes)


thread = Thread(target=update, daemon=True)
thread.start()
while True:
    map.animate(pose)
    sleep(1/60)
