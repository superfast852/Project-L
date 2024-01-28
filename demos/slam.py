from _pickle import load
from extensions.NavStack import Map, SLAM
import cv2


class lidarSim:
    def __init__(self, path="../Resources/test_scans.pkl", err=True):
        with open(path, "rb") as f:
            data = load(f)
            if len(data) == 2:
                self.map_meters, self.scans = data
            else:
                self.scans = data
                self.map_meters = 0
        self.count = 0
        self.err = err
    def read(self):
        try:
            scan = self.scans[self.count]
        except IndexError:
            if self.err:
                raise EOFError("No more scans available.")
            else:
                self.count = 0
                scan = self.scans[self.count]
        return scan


lidar = lidarSim()
map = Map(800)
slam = SLAM(None, map, lidar.map_meters if lidar.map_meters != 0 else 35)
for i in range(len(lidar.scans)):
    print(slam.px2pose(slam.update(lidar.read(), None)))
    img = map.tocv2(False)
    bot = slam.pose2cv2()
    img = cv2.arrowedLine(img, bot[0], bot[1], (0, 0, 255), 2)
    cv2.imshow("Map", img)
    cv2.waitKey(1)
cv2.destroyAllWindows()

