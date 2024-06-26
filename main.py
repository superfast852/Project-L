from extensions.logs import logging
from extensions.NavStack import Map, SLAM, np
from Robots.RM_HAL import RP_A1, IMU, MecanumKinematics, Drive
from extensions.XboxController import XboxController
from atexit import register
logger = logging.getLogger(__name__)

map = Map(800)
lidar = RP_A1()
imu = IMU()
kine = MecanumKinematics()
c = XboxController()
running = True
drive = Drive()
c.atloss = lambda: drive.brake()
slam = SLAM(lidar, map)

@register
def kill():
    global running
    running = False
    lidar.exit()
    map.save("./Resources/maps/main_scan_map.pkl")
    with open("./Resources/scan_registry.npy", "wb") as f:
        np.save(f, scan_registry)


scan_registry = []
last_scan = np.array(lidar.readCartesian())
c.setTrigger("Start", kill)
c.setTrigger("Back", drive.switchDrive)
logger.info("Starting main loop.")
while running:
    js = c.read()
    drive.drive(js[0], js[1], js[4], js[2])
    scan = lidar.getScan()
    scan_registry.append(scan)
    pose = slam.update(*scan)
    print(f"Pose: {pose}")



