from networktables import NetworkTables
from extensions.NavStack import Map
from threading import Thread
from time import sleep

map = Map(800)
driver_map = Map(800)
pose = (400, 400, 0)
driver_pose = (0, 0, 0)
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


def pose_update():
    global pose, driver_pose
    while not NetworkTables.isConnected():
        pass
    while True:
        map.fromSlam(bytearray(map_table.getRaw("map", bytes([127]*800*800))))
        pose = map_table.getNumberArray("pose", [0]*3)
        driver_pose = map_table.getNumberArray("dpose", [0]*3)
        sleep(0.5)


thread = Thread(target=pose_update, daemon=True)
thread.start()
while True:
    map.animate(pose=pose, show="slam")
    driver_map.animate(pose=driver_pose, show="driver")
    print(f"Pose Update:\n\t SLAM: {pose}\n\tDriver: {driver_pose}")
    sleep(0.5)
