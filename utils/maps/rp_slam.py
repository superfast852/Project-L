from SLAM import SLAM
from breezyslam.sensors import RPLidarA1
from rplidar import RPLidar as Lidar

map_px = "classroom.pkl"
map_m = 35
min_samples = 100
lidar = Lidar('/dev/ttyUSB0')
slam = SLAM(RPLidarA1(), map_px, map_m)

scan_iter = lidar.iter_scans()
prev_dists = None
prev_angles = None

print(next(scan_iter))
print(lidar.get_info())

try:
    while True:
        items = [item for item in next(scan_iter)]
        dists = [item[2] for item in items]
        angles = [item[1] for item in items]
        if len(dists) > min_samples:
            pose = slam.update(dists, angles=angles)
            prev_dists = dists.copy()
            prev_angles = angles.copy()

        elif prev_dists is not None:
            pose = slam.update(prev_dists, angles=prev_angles)
        else:
            pose = (0, 0, 0)
        print(pose)
except KeyboardInterrupt as f:
    print(f)
    print("Exiting...")
except Exception as e:
    print(f"Unknown Exception: {e}")


slam.save("classroom.pkl")
lidar.stop()
lidar.disconnect()