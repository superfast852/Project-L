from extensions.NavStack import Map
from Robots.RM_HAL import RP_A1, IMU, MecanumKinematics, Drive
from extensions.XboxController import XboxController
from atexit import register
from extensions.ICP import iterative_closest_point, transform_to_pose, transform_points, np

map = Map(800)
lidar = RP_A1()
imu = IMU()
kine = MecanumKinematics()
c = XboxController()
running = True
drive = Drive(collision_fn=lidar.autoStopCollision)
c.atloss = lambda: drive.brake()

@register
def kill():
    global running
    running = False
    lidar.exit()
    map.save()

def add_points(points):
    #map.map[points] = 1
    map.map[points[:, ::-1]] = 1


last_scan = np.array(lidar.readCartesian())
pose = np.array([0, 0, 0])
c.setTrigger("Start", kill)
c.setTrigger("Back", drive.switchDrive)
dev = 10
add_points(last_scan)
while running:
    js = c.read()
    drive.drive(js[0], js[1], js[4], js[2])
    scan = np.array(lidar.readCartesian())
    t = iterative_closest_point(scan, np.argwhere(map.map == 1), 100, 1e-3)
    # TODO: this absolutely could be wrong, i need to test total translation.
    pose += transform_to_pose(t)
    transformed = transform_points(t, scan)
    # Get the points that don't match
    distances = np.linalg.norm(transformed - last_scan, axis=1)
    add_points(scan[distances > dev])  # Add every point that is too far to be estimated as deviation.
    map.animate()





