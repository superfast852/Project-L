import cv2
import numpy as np
from breezyslam.algorithms import RMHC_SLAM
from Robots.RM_HAL import RP_A1, RPLidarException

lidar = RP_A1()

# Parameter Wall
slam = RMHC_SLAM(lidar, 800, 35)
map = bytearray(800 * 800)
try:
    while True:
        scan = lidar.getScan()
        slam.update(scan[0], None, scan[1])
        pose = slam.getpos()
        slam.getmap(map)
        img = np.array(map).reshape(800, 800)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.imshow("Map", img)
        cv2.waitKey(1)
except RPLidarException:
    pass
except KeyboardInterrupt:
    pass
finally:
    lidar.exit()