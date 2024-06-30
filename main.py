from extensions.logs import logging
from extensions.NavStack import Map, SLAM, np
from extensions.ICP import iterative_closest_point, transform_to_pose
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
icp_pose = np.array([0, 0, 0], dtype=np.float64)
dpose = [0, 0, 0]


@register
def kill():
    global running
    running = False
    lidar.exit()
    map.save("./Resources/maps/main_scan_map.pkl")


def update_map():
    global map_table, map
    while not NetworkTables.isConnected():
        sleep(1/5)
    while running:
        map_table.putNumberArray("icp_pose", list(icp_pose))
        map_table.putNumberArray("dpose", driver.pose.copy())
        map_table.putNumberArray("pose", pose)
        sleep(0.5)


def toCart(d, a):
    dist, a = np.array([d, a])
    angle = np.deg2rad(a)
    return np.array([dist*np.cos(angle), dist*np.sin(angle)]).T


lastScan = None
c.setTrigger("Start", kill)
c.setTrigger("Back", drive.switchDrive)
logger.info("Starting main loop.")
Thread(target=update_map, daemon=True).start()
while running:
    js = c.read()
    drive.drive(js[1], js[0], js[4], js[2])
    scan = lidar.getScan()
    pose = slam.update(*scan)

    cart_points = toCart(*scan)
    if lastScan is not None:
        t = iterative_closest_point(cart_points, lastScan, 100, 0.001)
        icp_pose += transform_to_pose(t)
    lastScan = cart_points.copy()
