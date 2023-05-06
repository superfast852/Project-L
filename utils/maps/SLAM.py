from breezyslam.algorithms import RMHC_SLAM
from roboviz import MapVisualizer
from _pickle import load, dump
from numpy import array, argwhere, logical_not, ndarray
from rrtplanner.oggen import perlin_occupancygrid
# TODO: Default unit for movement should be pixels


def generate_random_map(map_size):
    map = perlin_occupancygrid(map_size, map_size)
    map = logical_not(map).astype(int)
    for index in argwhere(map == 1):
        map[index[0]][index[1]] = 255
    return bytearray(map.flatten().tolist())


class SLAM:
    def __init__(self, lidar, map, map_meters=None, update=1, graph=1):
        #  Map Extraction. Can be pkl file, bytearray, or int
        if isinstance(map, str):
            if not map.endswith(".pkl"):
                raise TypeError("map file must be pkl")
            with open(map, "rb") as f:
                data = load(f)
                if len(data) == 2:
                    if map_meters is None:
                        map_meters = data[0]
                        print(map_meters)
                    data = data[1]
                map = data
                self.map_size = int(len(map)**0.5)
                print("Loaded via pkl!")
        if isinstance(map, bytearray):
            self.map_size = len(map)**0.5
            self.map = map
        elif isinstance(map, int):
            self.map_size = map
            self.map = bytearray(map**2)
        elif isinstance(map, ndarray):
            print("Converting ndarray to bytearray")
            self.map_size = len(map)**0.5
            self.map = bytearray(map.flatten().tolist())
        else:
            raise TypeError("map must be bytearray or int")

        self.lidar = lidar
        self.ShouldUpdate = update
        self.graph = graph
        self.map_size = int(self.map_size)
        self.slam = RMHC_SLAM(lidar, self.map_size, map_meters)
        self.slam.setmap(self.map)
        if self.graph:
            self.viz = MapVisualizer(self.map_size, map_meters, "SLAM")
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

    def save(self, name=None):
        if name is None:
            from datetime import datetime
            datetime = datetime.today()
            name = f"{datetime.day}-{datetime.month}-{datetime.year} {datetime.hour}:{datetime.minute}.pkl"
        with open(name, "wb") as f:
            dump((self.map_size_meters, self.map), f)

    def MapToXY(self):
        map = array(self.map)
        return map.reshape((int(len(map)**0.5), int(len(map)**0.5)))

    @staticmethod
    def XYToBytes(map):
        # Fix This!!!
        return bytearray(map.flatten().tolist())

    def toPath(self, filter=200):
        map = self.MapToXY()
        # Turn to Binary:
        for index in argwhere(map > filter):
            map[index[0]][index[1]] = 255
        for index in argwhere(map <= filter):
            map[index[0]][index[1]] = 0
        # Turn to 0s and 1s:
        # rrtplanner Expects a 1 as an obstacle and 0 as an empty space.
        for index in argwhere(map == 255):
            map[index[0]][index[1]] = 1
        map = logical_not(map).astype(int)
        return map

if __name__ == "__main__":
    from extensions.HAL import LD06
    from PathPlanner import Planner

    # Parameters
    feed_scan = 1
    map_data = "4-5-2023 15:51.pkl"
    save = 0

    # Objects
    laser = LD06()
    slam = SLAM(laser, map_data, 35)
    pathplan = Planner(slam.map)
    try:
        while True:
            if feed_scan:
                # scan = lidar.read()
                # pose = slam.update(scan)
                distances, thetas = laser.read(return_angle=1)
                coords = slam.update(distances.tolist(), angles=thetas)
            else:
                coords = slam.update()
    except KeyboardInterrupt as k:
        print(k)
        print("Exiting...")

    if save:
        slam.save("classroom.pkl")
        print("Saved!")
