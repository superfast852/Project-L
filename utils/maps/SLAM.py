from breezyslam.algorithms import RMHC_SLAM
from roboviz import MapVisualizer
from _pickle import load, dump
from numpy import array


class SLAM:
    def __init__(self, lidar, map, map_meters, update=1, graph=1):
        #  Map Extraction. Can be pkl file, bytearray, or int
        if isinstance(map, str):
            if not map.endswith(".pkl"):
                raise TypeError("map file must be pkl")
            with open(map, "rb") as f:
                data = load(f)
                if len(data) == 2:
                    if map_meters is None:
                        map_meters = data[0]
                    data = data[1]
                map = data
        if isinstance(map, bytearray):
            map_size = len(map)**0.5
            self.map = map
        elif isinstance(map, int):
            map_size = map
            self.map = bytearray(map**2)
        else:
            raise TypeError("map must be bytearray or int")

        self.lidar = lidar
        self.ShouldUpdate = update
        self.graph = graph
        self.slam = RMHC_SLAM(lidar, map_size, map_meters)
        self.slam.setmap(self.map)
        if self.graph:
            self.viz = MapVisualizer(map_size, map_meters, "SLAM")
        self.map_size_meters = map_meters

    def update(self, scan=None, angles=None):
        if angles is not None and scan is not None:
            self.slam.update(scan, scan_angles_degrees=angles, should_update_map=self.ShouldUpdate)
        else:
            if scan is None:
                scan = self.lidar.read()
            self.slam.update(scan, should_update_map=self.ShouldUpdate)
        pose = self.slam.getpos()
        self.slam.getmap(self.map)
        if self.graph:
            if not self.viz.display(pose[0]/1000, pose[1]/1000, pose[2], self.map):
                raise KeyboardInterrupt("Map Closed")
        return pose

    def save(self, name):
        with open(name, "wb") as f:
            dump((self.map_size_meters, self.map), f)

    def MapToXY(self):
        map = array(self.map)
        return map.reshape((int(len(map)**0.5), int(len(map)**0.5)))

    @staticmethod
    def XYToBytes(map):
        # Fix This!!!
        return bytearray(map.flatten)


if __name__ == "__main__":
    from HAL import LD06

    feed_scan = 1
    save = 0
    lidar = LD06()
    slam = SLAM(lidar, 800, 35)

    try:
        while True:
            if feed_scan:
                scan = lidar.read()
                pose = slam.update(scan)
                #distances, angles = lidar.read(map_or_angle=1)
                #pose = slam.update(distances, angles=angles)
            else:
                pose = slam.update()
    except KeyboardInterrupt as f:
        print(f)
        print("Exiting...")

    if save:
        slam.save("map.pkl")
