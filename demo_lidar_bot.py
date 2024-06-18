from extensions.XboxController import XboxController
from extensions.logs import logging
from Robots.RM_HAL import Drive, RP_A1, MecanumKinematics
from extensions.NavStack import SLAM, Map
from time import sleep
from networktables import NetworkTables
import atexit
import signal
import datetime
now = datetime.datetime.now
logger = logging.getLogger(__name__)

drive = Drive()
map = Map(800)
lidar = RP_A1(threaded=True)
slam = SLAM(lidar, map)
kine = MecanumKinematics()

# Unity Visualization
NetworkTables.initialize()
maptable = NetworkTables.getTable("map")
while maptable.getNumber("meters", 0) == 0:
    maptable.putNumber("meters", map.map_meters)
botTable = NetworkTables.getTable("bot")
while botTable.getNumber("size", 0) == 0:
    botTable.putNumber("size", 3)

while True:
    try:
        controller = XboxController()
        break
    except OSError:
        pass

killsig = False


def kill(*args, **kwargs):
    global killsig
    if killsig:
        return None
    drive.brake()
    lidar.exit()
    drive.exit()
    NetworkTables.stopServer()
    NetworkTables.shutdown()
    logger.info(f"[{__name__}]Exited gracefully.")
    killsig = True


controller.setTrigger("Back", kill)
controller.setTrigger("Start", drive.switchDrive)
atexit.register(kill)
signal.signal(signal.SIGTERM, kill)
signal.signal(signal.SIGINT, kill)

while True:
    try:
        vals = controller.read()
        drive.drive(vals[0], vals[1], vals[4], vals[2])
        if killsig:
            break
        distances, angles = lidar.latest
        pose = slam.update(distances, angles)
        maptable.putRaw("data", map.toSlam())
        botTable.putNumberArray("pose", pose)
        botTable.putNumberArray("kine", kine.pose)
        sleep(1/60)
    except Exception as e:
        logger.error(f"{__name__}: {e}\n{e.args}")
        kill()
logger.info(f"Ended at {now()}\n\n")
