from SLAM import SLAM
from breezyslam.sensors import RPLidarA1
from rp_slam import RPLidar as Lidar

map_px = 800
map_m = 10
min_samples = 200
lidar = Lidar('/dev/ttyUSB0', baudrate=115200, timeout=3)
slam = SLAM(RPLidarA1(), map_px, map_m)

scan_iter = lidar.iter_scans()
prev_dists = None
prev_angles = None

next(scan_iter)

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

        print(pose)
except KeyboardInterrupt as f:
    print(f)
    print("Exiting...")
except Exception as e:
    print(f"Unknown Exception: {e}")


slam.save("map.pkl")
lidar.stop()
lidar.disconnect()