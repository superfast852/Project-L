from networktables import NetworkTables
from API import RP_A1, SLAM, Map, Drive, MecanumKinematics, XboxController, driver
import atexit
import signal
from time import sleep

drive = Drive()
map = Map(800)
lidar = RP_A1(threaded=True)
slam = SLAM(lidar, map)
kine = MecanumKinematics()


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
    print("Exited gracefully.")
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
        if controller.Y:
            driver.set_beep(10)
        distances, angles = lidar.latest
        pose = slam.update(distances, angles)
        maptable.putRaw("data", map.toUnity())
        botTable.putNumberArray("pose", pose)
        botTable.putNumberArray("kine", kine.pose)
        sleep(1/60)
    except Exception as e:
        print(f"[ERROR] demo_bot: {e}\n{e.args}")
        kill()