# NOTE: The code crashing is due to the IDE, not the code itself.
# SLAM
# TODO: we import a shit ton of stuff from numpy and then numpy as np. 0 sense.
from __future__ import annotations


# Path Planning
try:
    from extensions.rrt_backend import RRTStarInformed, random_point_og  # TODO: Find something else, dude.
    from extensions.tools import evaluate_bezier, line2dots, ecd, getAngle
except ModuleNotFoundError:
    from rrt_backend import RRTStarInformed, random_point_og
    from tools import evaluate_bezier, line2dots, ecd, getAngle

# Other utilities
from breezyslam.algorithms import RMHC_SLAM
from _pickle import dump, load
from numpy import array, argwhere, logical_not, ndarray, max, load, rot90, uint8, cos, sin, linspace, random, append as npappend, float32
from numpy.linalg import norm as linalg_norm
from cv2 import cvtColor, COLOR_GRAY2BGR, line as cvline, circle as cvcircle, arrowedLine, imshow, waitKey
from time import time


def pymap(n, func):
    return map(func, n)


class Map:
    paths = []

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
                    self.__init__(values)
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
        self.map_center = [i//2 for i in self.map.shape]

    def update(self, map):
        self.__init__(map)

    def fromSlam(self, map):
        size = int(len(map) ** 0.5)  # get the shape for a square map (1d to 2d)
        # convert from bytearray to 2d np array, apply quality threshold, scale down to 0-1, reshape
        map = ((array(map)-73)/255).reshape(size, size).round()
        # the threshold value (73) is obtained like this: (thresval-127) where thresval is the cutoff in 0-255 scale.
        # after subtracting, only values above thresval remain above 0.5. When we round, only those turn to 1.
        self.map = logical_not(map).astype(int)  # Then we apply a logical not to comply with the RRT map.

    def algocp(self, map):
        size = int(len(map) ** 0.5)
        map = (array(map).reshape(size, size) / 255 - 0.284313725).round()
        # Finally we invert the map, so 0 is free space and 1 is occupied space.
        self.map = logical_not(map).astype(int)

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

    def drawPoint(self, img, point, r=2, c=(0, 0, 255), t=2):
        return cvcircle(img, (point[0], self.map.shape[0]-point[1]), r, c, t)

    def drawPx(self, img, point, c=(0, 0, 255), r=1):
        for a in range(-r, r):
            for b in range(-r, r):
                img[self.map.shape[0]-(point[1]+b)][point[0]+a] = c
        return img

    def drawLine(self, img, line, c=(0, 255, 0), **kwargs):
        return cvline(img, (line[0][0], self.map.shape[0]-line[0][1]),
                        (line[1][0], self.map.shape[0]-line[1][1]), c, **kwargs)

    def drawLineOfDots(self, img, line, c=(0, 255, 0)):
        [self.drawLine(img, (line[i], line[i+1]), c=c, thickness=2) for i in range(len(line)-1)]

    def getValidRoute(self, n):
        return [self.getValidPoint() for _ in range(n)]

    def _posearrowext(self, pose, r):
        return pose[:2], (round(pose[0]+r*cos(pose[2])), round(pose[1]+r*sin(pose[2])))

    def animate(self, img=None, pose=None, drawLines=True):
        # Pose is expected to be 2 coordinates, which represent a center and a point along a circle.
        if img is None:
            img = self.tocv2()
        if drawLines:
            for path in self.paths:
                try:
                    path = path.tolist()
                except AttributeError:
                    pass
                if path:
                    img = self.drawPoint(img, path[0][0], 4)
                    img = self.drawPoint(img, path[-1][1], 4)
                    color = random.randint(0, 256, 3).tolist()
                    for line in path:
                        img = self.drawLine(img, line, color)
        if pose is not None:
            if pose == "center":
                arrowedLine(img, self.map_center, tuple(pymap(self.map_center, lambda x: x-5)), (0, 0, 255), 3)
            else:
                pt1, pt2 = self._posearrowext(pose, 20)
                arrowedLine(img, pt1, pt2, (0, 0, 255), 3)
        imshow("Map", img)
        waitKey(1)

    def addPath(self, route):
        # TODO: Fix this. it can be brokey
        # A chain has 3 dimensions, sorta, while a path has 2
        # In a chain, D[0] = path, D[1] = line, D[2] = Pixel
        # In a path,  D[0] = line, D[1] = Pixel
        j = route[0]  # First Dimension
        i = 0
        while isinstance(j, list) or isinstance(j, ndarray):
            i += 1
            if j != []:
                try:
                    j = j[0]
                except IndexError:
                    raise IndexError(f"IndexError! Offending path: {route}")
            else:

                break
        if i == 3:
            for path in route:
                self.paths.append(path)
        elif i == 2:
            self.paths.append(route)
        else:
            print("Found dead link.")
            # raise ValueError("Could not determine if route is a chain or path.")


class SLAM:
    def __init__(self, lidar=None, map_handle=None, map_meters=35, update_map=1):
        self.map = map_handle if map_handle else Map(800)
        if lidar is None:
            from breezyslam.sensors import RPLidarA1
            lidar = RPLidarA1()
        self.mapbytes = self.map.toSlam()
        self.ShouldUpdate = update_map
        self.map_size = len(self.map)  # Ensure it's actually an integer, might've been calculated.
        self.pose = (0, 0, 0)
        # Slam Preparation
        self.ratio = map_meters*1000 / self.map_size  # For converting MM to px.
        self.slam = RMHC_SLAM(lidar, self.map_size, map_meters)
        self.slam.setmap(self.mapbytes)

    def update(self, distances, angles=None, odometry=None):
        # You can choose what angles to feed the slam algo. That way, if the lidar's blocked, you can filter it.
        if angles is None:
            angles = list(range(360))  # we assume it's a premade map of 360 degrees (idiotic btw, dont use this.)
        elif not isinstance(angles, list):
            angles = list(angles)  # Ensure that angles is a list

        if len(angles) > len(distances):
            angles = angles[:len(distances)]
        elif len(distances) > len(angles):
            distances = distances[:len(angles)]
        self.slam.update(distances, odometry, angles, self.ShouldUpdate)
        # keep in mind that SLAM basically remains running on its own map. So no need to worry about data loss
        self.slam.getmap(self.mapbytes)  # as seen here, SLAMOps run on self.mapbytes exclusively.
        self.map.fromSlam(self.mapbytes)  # then, it gets exported to self.map
        self.pose = self.pose2px(self.slam.getpos())
        return self.pose  # SLAM

    def pose2px(self, pose=None):
        return round(pose[0] / self.ratio), round(pose[1] / self.ratio), pose[2]  # theta is just rotation so it's fine

    def px2pose(self, px=None):
        return px[0] * self.ratio, px[1] * self.ratio, px[2] if len(px) == 3 else None

    def pose2cv2(self, size=10):  # TF was i tryna do?
        x, y, r = self.pose  # in px
        return (x, y), (int(x + size * cos(r)), int(y + size * sin(r)))


class RRT:
    # TODO: Deal with dead links. (Dead links are paths that couldnt be found. They're returned as empty lists.)
    # TODEAL: Make a function to find dead links in a path or chain, i guess
    def __init__(self, map: Map, n=1000, obstacle_distance=100, goal_radius=50, pbar=False):
        self.n = n
        self.r_rewire = obstacle_distance
        self.r_goal = goal_radius
        self.pbar = pbar
        self.map = map
        self.planner = RRTStarInformed(self.map.map, self.n, self.r_rewire, self.r_goal, pbar=self.pbar)

    def plan(self, start: list | tuple, goal: list | tuple):
        # If it isn't, update the preset object.
        xgoal = goal
        xstart = start
        found = 0
        lines = []
        while not found:
            if not self.map.isValidPoint(xgoal):
                print(f"[ERROR] Planner: Goal at {xgoal} is an Obstacle!")
                return None  # None indicates no point could be found. We avoid computation that way.

            # Create planner with latest map.
            # self.planner.set_og(map.map)
            # Planner asks for coords to be arrays, so we convert them. Also make sure they're 2D.
            start = array(xstart[0:2])
            goal = array(xgoal[0:2])
            if self.planner.collisionfree(self.map.map, start, goal):
                return array([[start, goal]])

            tree, route = self.planner.plan(start, goal)  # Execute RRT* Informed.
            if route is None:
                print("[ERROR] Planner: No path was found! Try increasing iterations or decreasing rewire radius.")
                return None
            lines = self.planner.vertices_as_ndarray(tree, self.planner.route2gv(tree, route))  # Convert route to lines.
            # The lines are composed by arrays in form [[start point, end point], [start point, end point], ...]
            # So we'll just extract every end point to get the waypoints.
            # Remember, these are pixels.
            del tree, route, start, goal  # Delete planner and map to free up memory.
            if [] in lines:
                print("Found Empty Coords!")
                print(lines)
            if isinstance(lines, ndarray):
                if lines.size != 0:
                    found = 1
            elif isinstance(lines, list):
                if lines:
                    found = 1

        return lines

    @staticmethod
    def getEndPoints(lines):
        lines = array(lines)
        return npappend([lines[0][0]], lines[:, 1], 0)

    # Note: Either im a shitty dev or sequential is faster than parallel
    def chainroute(self, points: list, one_dim=False):
        out = []
        for i in range(len(points)-1):
            path = self.plan(points[i], points[i+1])
            if path == []:
                print("Invalid path found.")
            if one_dim:
                out += path
            else:
                out.append(path)
        return out

    def smoothPath(self, path, n=50):
        endpoints = npappend([path[0][0]], path[:, 1], 0)
        return evaluate_bezier(endpoints, n).astype(int)

    def find_corridor(self, path, lsr=50, furthest=False, isDots=False):
        corridors = []
        for line in path:
            if not isDots:
                dotted_line = line2dots(line)
            else:
                dotted_line = line
            corridor = []
            for dot in dotted_line:
                x, y = dot
                obstacle = [0, 0]
                # TODO: make it so that the returned point is just the one with the furthest obstacle.
                for i in range(lsr):
                    if furthest:
                        if 0 not in obstacle:
                            break
                    # Right Sided Distance:
                    try:  # Save the first obstacle found in lateral search
                        if obstacle[1] == 0 and self.map.map[x + i][y]:
                            obstacle[1] = i
                        # Left Sided Distance
                        if obstacle[0] == 0 and self.map.map[x - i][y]:
                            obstacle[0] = i
                    except IndexError:
                        pass
                a, b = obstacle
                if furthest:
                    if not a or not b:
                        corridor.append((x, y))
                    elif a > b:
                        corridor.append((x - a, y))
                    else:
                        corridor.append((x + b, y))
                else:
                    if not a or not b:
                        corridor.append((x, y))
                    elif a > b:
                        corridor.append((x + b, y))
                    else:
                        corridor.append((x - a, y))
            corridors.append(corridor)
        return corridors


class PathFollow:
    def __init__(self, path, r=50):
        # self.path = npappend(path[:, 0], [path[-1][1]], axis=0)
        self.path = array([points[1] for points in path])
        self.r = r
        self.current_waypoint = self.path[0]

    def __call__(self, pose):
        if self.current_waypoint is None:  # Checking if it has finished
            return 0, 0, 0
        # Splitting the pose for convenience
        pose = array(pose)
        xy = pose[:2]
        z = pose[2]
        print(xy, self.current_waypoint)
        distance = ecd(xy.astype(float32), self.current_waypoint.astype(float32))

        if distance < self.r:  # Consider as arrived
            ind = argwhere(self.path == self.current_waypoint).flatten()[0]+1  # Get the index of the next waypoint
            if ind >= len(self.path):
                self.current_waypoint = None
                return 0, 0, 0
            self.current_waypoint = self.path[ind]  # Set waypoint

        diff_xy = self.current_waypoint - xy # Get point difference
        # Regulate to always be capped at 1 in any direction
        norm = linalg_norm(diff_xy)
        reg_xy = diff_xy/norm if norm != 0 else diff_xy  # Note: the abs might fuck things up. TODO: Please check.

        # turn calculations: take for future reference
        desired_t = getAngle(*diff_xy)  # get angle of point
        ang_diff = desired_t-z  # get angle_spacing
        # wrapping
        while ang_diff > 3.14159265:
            ang_diff -= 2 * 3.14159265
        while ang_diff < -3.14159265:
            ang_diff += 2 * 3.14159265

        # theta_diff = self._turnCalc(pose, reg_xy)
        return *reg_xy, max(min(ang_diff, 1), -1)

    def getMovement(self, pose):
        if self.current_waypoint is None:  # Checking if it has finished
            return 0, 0, 0
        # Splitting the pose for convenience
        pose = array(pose)
        xy = pose[:2]
        z = pose[2]
        print(xy, self.current_waypoint)

        # Waypoint picking
        distance = ecd(xy.astype(float32), self.current_waypoint.astype(float32))

        if distance < self.r:  # Consider as arrived
            ind = argwhere(self.path == self.current_waypoint).flatten()[0]+1  # Get the index of the next waypoint
            if ind >= len(self.path):
                self.current_waypoint = None
                return 0, 0, 0
            self.current_waypoint = self.path[ind]  # Set waypoint

        # Movement Calculations
        diff_xy = self.current_waypoint - xy
        desired_t = getAngle(*diff_xy)  # get angle of point
        ang_diff = desired_t - z  # get angle_spacing
        # wrapping
        while ang_diff > 3.14159265:
            ang_diff -= 2 * 3.14159265
        while ang_diff < -3.14159265:
            ang_diff += 2 * 3.14159265

        return cos(ang_diff), sin(ang_diff), max(min(ang_diff, 1), -1)

    @staticmethod
    def _turnCalc(pose, target):
        # Calculate the desired orientation for the robot to point to the target
        desired_orientation = getAngle(target[0] - pose[0], target[1] - pose[1])
        # Calculate the angular difference between the robot's current orientation and the desired orientation
        angular_difference = desired_orientation - pose[2]
        # Adjust the angular difference for wrap-around at 2*pi
        while angular_difference > 3.14159265:
            angular_difference -= 2 * 3.14159265
        while angular_difference < -3.14159265:
            angular_difference += 2 * 3.14159265
        # Determine the turn command based on the angular difference (reversed directions)
        return max(min(-angular_difference, 1), -1)
