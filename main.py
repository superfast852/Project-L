from extensions.logs import logging
from extensions.NavStack import Map, SLAM, np
from Robots.RM_HAL import RP_A1, IMU, Drive, driver
from extensions.XboxController import XboxController
from networktables import NetworkTables
from threading import Thread
from atexit import register
from time import sleep
logger = logging.getLogger(__name__)

map = Map(800)
lidar = RP_A1()
imu = IMU()
c = XboxController()
running = True
drive = Drive()
c.atloss = lambda: drive.brake()
slam = SLAM(lidar, map)
NetworkTables.initialize()
map_table = NetworkTables.getTable("map")
map_table.putRaw("map", map.toSlam())
pose = (400, 400, 0)


@register
def kill():
    global running
    running = False
    lidar.exit()
    map.save("./Resources/maps/main_scan_map.pkl")
    with open("./Resources/scan_registry.npy", "wb") as f:
        np.save(f, scan_registry)


def update_map():
    global map_table, map
    while not NetworkTables.isConnected():
        sleep(1/5)
    while running:
        map_table.putRaw("map", map.toSlam())
        map_table.putNumberArray("dpose", driver.pose)
        map_table.putNumberArray("pose", pose)
        sleep(1/10)


scan_registry = []
c.setTrigger("Start", kill)
c.setTrigger("Back", drive.switchDrive)
logger.info("Starting main loop.")
Thread(target=update_map, daemon=True).start()
while running:
    js = c.read()
    drive.drive(js[0], js[1], js[4], js[2])
    scan = lidar.getScan()
    scan_registry.append(scan)
    pose = slam.update(*scan)
