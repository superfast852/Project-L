from networktables import NetworkTables
from extensions.NavStack import Map
from threading import Thread
from time import sleep

map = Map(800)
pose = (400, 400, 0)
NetworkTables.initialize("orinnano.local")
map_table = NetworkTables.getTable("map")
NetworkTables.initialize()


def update():
    global map_table, map, pose
    while True:
        mapbytes = map_table.getRaw("map", bytearray([127]*800*800))
        pose = list(map_table.getNumberArray("pose", (400, 400, 0)))
        pose[2] *= 3.14159265/180
        map.fromSlam(bytearray(mapbytes))


thread = Thread(target=update, daemon=True)
thread.start()
while True:
    map.animate(pose=pose)
    sleep(1/60)
