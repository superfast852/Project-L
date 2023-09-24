# TODO: Implement matplotlib animation. (https://pythonprogramming.net/live-graphs-matplotlib-tutorial/)
# SLAM
from __future__ import annotations

from breezyslam.algorithms import RMHC_SLAM

# Path Planning
try:
    from extensions.rrt_backend import RRTStarInformed, random_point_og  # TODO: Find something else, dude.
except ModuleNotFoundError:
    from rrt_backend import RRTStarInformed, random_point_og

# Other utilities
from _pickle import dump, load
from numpy import array, argwhere, logical_not, ndarray, max, load, rot90, uint8, cos, sin, linspace, random
from cv2 import cvtColor, COLOR_GRAY2BGR, line as cvline, circle as cvcircle
from threading import Thread

class Map:
    def __init__(self, map):
        # The IR Map is just the RRT Map format.

        if isinstance(map, str):
            if map.lower() == "random":
                # Get a randomly generated map for testing.
                try:
                    self.map = load("./Resources/map.npy")  # Formatted as an RRT Map.
                except FileNotFoundError:
                    self.map = load("../Resources/map.npy")
            elif "pkl" in map:
                with open(map, "rb") as f:
                    values = load(f, allow_pickle=True)
                    if len(values) == 2 and (isinstance(values, tuple) or isinstance(values, list)):
                        size, bytearr = values
                        print(size)
                        map = array(bytearr).reshape(int(len(bytearr) ** 0.5), int(len(bytearr) ** 0.5))
                        for index in argwhere(map < 200):  # First we pull down the readings below quality.
                            map[index[0], index[1]] = 0
                        for index in argwhere(
                                map >= 200):  # Then we pull up the readings above quality and convert to 1.
                            map[index[0], index[1]] = 1
                        # Finally we invert the map, so 0 is free space and 1 is occupied space.
                        self.map = logical_not(map).astype(int)
                        self.map_size = size
                    else:
                        if isinstance(values, ndarray):
                            self.map = values
                        else:
                            raise ValueError("Could not extract map from .pkl file.")

        elif isinstance(map, bytearray):
            # convert from slam to IR
            # Slam maps are bytearrays that represent the SLAM Calculated map. Higher the pixel value, clearer it is.
            map = array(map).reshape(int(len(map) ** 0.5), int(len(map) ** 0.5))
            for index in argwhere(map < 200):  # First we pull down the readings below quality.
                map[index[0], index[1]] = 0
            for index in argwhere(map >= 200):  # Then we pull up the readings above quality and convert to 1.
                map[index[0], index[1]] = 1
            # Finally we invert the map, so 0 is free space and 1 is occupied space.
            self.map = logical_not(map).astype(int)

        elif isinstance(map, ndarray):
            # convert from rrt to IR
            # RRT Maps are numpy arrays that RRT uses to calculate a route to a point. 1 represents an obstacle.
            if max(map) > 1:
                for index in argwhere(map < 200):  # First we pull down the readings below quality.
                    map[index[0], index[1]] = 0
                for index in argwhere(map >= 200):  # Then we pull up the readings above quality and convert to 1.
                    map[index[0], index[1]] = 1
                # Finally we invert the map, so 0 is free space and 1 is occupied space.
                self.map = logical_not(map).astype(int)
            else:  # If we get an IR Map, we just use it.
                self.map = map

        elif isinstance(map, int):
            # generate a blank IR Map
            self.map = array([0]*map**2).reshape(map, map)

        else:
            raise ValueError("Map format unidentified.")

    def update(self, map):
        self.__init__(map)

    def toSlam(self):
        return bytearray([(not i)*255 for i in self.map.flatten()])

    def save(self, name=None):
        if name is None:
            from datetime import datetime
            datetime = datetime.today()
            name = f"{datetime.day}-{datetime.month}-{datetime.year} {datetime.hour}:{datetime.minute}.pkl"
            del datetime
        with open(name, "wb") as f:
            dump(self.map, f)
        print(f"[INFO] Saved Map as {name}!")

    def isValidPoint(self, point):
        return not self.map[point[0], point[1]]

    def getValidPoint(self) -> tuple:
        return tuple(random_point_og(self.map))

    def __len__(self):
        return len(self.map)

    def tocv2(self, invert=True):
        map = self.map if not invert else logical_not(self.map)
        return cvtColor(rot90(map).astype(uint8) * 255, COLOR_GRAY2BGR)

    def drawPoint(self, img, point, r=2, c=(0,0,255), t=2):
        return cvcircle(img, (point[0], self.map.shape[0]-point[1]), r, c, t)

    def drawLine(self, img, line, c=(0, 255, 0)):
        return cvline(img, (line[0][0], self.map.shape[0]-line[0][1]),
                        (line[1][0], self.map.shape[0]-line[1][1]), c)

    def getValidRoute(self, n):
        return [self.getValidPoint() for _ in range(n)]


class SLAM:
    def __init__(self, lidar, map_handle, map_meters, update_map=1):
        self.map = map_handle
        self.mapbytes = self.map.toSlam()
        self.ShouldUpdate = update_map
        self.map_size = len(self.map)  # Ensure it's actually an integer, might've been calculated.
        self.pose = (0, 0, 0)
        # Slam Preparation
        self.ratio = map_meters*1000 / self.map_size  # For converting MM to px.
        self.slam = RMHC_SLAM(lidar, self.map_size, map_meters)
        self.slam.setmap(self.mapbytes)

    def update(self, distances, angles):
        # You can choose what angles to feed the slam algo. That way, if the lidar's blocked, you can filter it.
        if angles is None:
            angles = linspace(0, 360, len(distances)).tolist()
        else:
            angles = list(angles)  # Ensure that angles is a list
        if len(angles) > len(distances):
            angles = angles[:len(distances)]
        elif len(distances) > len(angles):
            distances = distances[:len(angles)]
        self.slam.update(distances, scan_angles_degrees=angles, should_update_map=self.ShouldUpdate)
        self.slam.getmap(self.mapbytes)
        self.map.update(self.mapbytes)
        self.pose = self.pose2px(self.slam.getpos())
        return self.pose  # SLAM

    def pose2px(self, pose=None):
        return round(pose[0] / self.ratio), round(pose[1] / self.ratio), pose[2]  # theta is just rotation so it's fine

    def px2pose(self, px=None):
        return px[0] * self.ratio, px[1] * self.ratio, px[2] if len(px) == 3 else None

    def pose2cv2(self, size=10):  # TF was i tryna do?
        x, y, r = self.pose  # in px
        color = (0, 0, 255)
        end_point = (int(x + size * cos(r)), int(y + size * sin(r)))


class RRT:
    def __init__(self, n=250, obstacle_distance=100, goal_radius=50, pbar=False):
        self.n = n
        self.r_rewire = obstacle_distance
        self.r_goal = goal_radius
        self.pbar = pbar
        self.planner = RRTStarInformed(map.map, self.n, self.r_rewire, self.r_goal, pbar=self.pbar)

    def plan(self, start: list | tuple, goal: list | tuple, map: Map):
        # TODO: If the map is None, use a preset RRT object.
        # If it isn't, update the preset object.
        if not map.isValidPoint(goal):
            print(f"[ERROR] Planner: Goal at {goal} is an Obstacle!")
            return None  # None indicates no point could be found. We avoid computation that way.

        # Create planner with latest map.
        self.planner.set_og(map.map)
        # Planner asks for coords to be arrays, so we convert them. Also make sure they're 2D.
        start = array(start[0:2])
        goal = array(goal[0:2])

        tree, route = self.planner.plan(start, goal)  # Execute RRT* Informed.
        if route is None:
            print("[ERROR] Planner: No path was found! Try increasing iterations or decreasing rewire radius.")
            return None
        lines = self.planner.vertices_as_ndarray(tree, self.planner.route2gv(tree, route))  # Convert route to lines.
        # The lines are composed by arrays in form [[start point, end point], [start point, end point], ...]
        # So we'll just extract every end point to get the waypoints.
        # Remember, these are pixels.
        del tree, route, start, goal  # Delete planner and map to free up memory.
        return lines

    @staticmethod
    def getWaypoints(lines):
        return [i[1] for i in lines]

    # Note: Either im a shitty dev or sequential is faster than parallel
    def chainroute(self, points: list, map: Map, one_dim=False):
        out = []
        for i in range(len(points)-1):
            path = self.plan(points[i], points[i+1], map)
            if one_dim:
                out += path
            else:
                out.append(path)
        return out

    def chainroute_parallel(self, points: list, map: Map, one_dim=False):
        buffer = {key: None for key in range(len(points)-1)}

        def tempfunc(n):
            buffer[n] = self.plan(points[n], points[n+1], map).tolist()

        for i in range(len(points)-1):
            Thread(target=tempfunc, args=(i,)).start()
        while None in list(buffer.values()):
            pass

        if one_dim:
            out = []
            for i in list(buffer.values()):
                out += i
        else:
            out = list(buffer.values())
        # TODO: add simultaneous choice (maybe as a custom object/type?)
        return out


if __name__ == "__main__":
    from cv2 import imshow, waitKey

    refresh = 1
    single = 0
    map = Map("random")
    image = map.tocv2()
    imshow("Binary Image", image)
    planner = RRT()
    while True:
        if refresh:
            image = map.tocv2()
        if single:
            start = map.getValidPoint()
            stop = map.getValidPoint()
            path = planner.plan(start, stop, map)
            image = map.drawPoint(image, start)
            image = map.drawPoint(image, stop)
            for line in path:
                map.drawLine(image, line)
        else:
            route = map.getValidRoute(5)
            for point in route:
                image = map.drawPoint(image, point)
            chain = planner.chainroute(route, map)
            for paths in chain:
                for line in paths:
                    image = map.drawLine(image, line, random.randint(0, 256, 3).tolist())


        # Display the image using OpenCV
        imshow("Binary Image", image)
        waitKey(1)
        # sleep(1)
