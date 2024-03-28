from __future__ import annotations
from dataclasses import dataclass
from breezyslam.algorithms import RMHC_SLAM
from pickle import dump, load
import numpy as np
import cv2


def pymap(n, func):
    return map(func, n)


class Map:
    paths = []

    def __init__(self, map, map_meters=35):
        # The IR Map is just the RRT Map format.
        self.map_meters = None
        if isinstance(map, str):
            if map.lower() == "random":
                # Get a randomly generated map for testing.
                try:
                    self.map = np.load("./Resources/map.npy")  # Formatted as an RRT Map.
                except FileNotFoundError:
                    self.map = np.load("../Resources/map.npy")
                self.map_meters = 35
            elif "pkl" in map:
                with open(map, "rb") as f:
                    values = load(f)
                if len(values) == 2 and (isinstance(values, tuple) or isinstance(values, list)):
                    meters, bytearr = values
                    map = np.array(bytearr).reshape(int(len(bytearr) ** 0.5), int(len(bytearr) ** 0.5))
                    for index in np.argwhere(map < 200):  # First we pull down the readings below quality.
                        map[index[0], index[1]] = 0
                    for index in np.argwhere(
                            map >= 200):  # Then we pull up the readings above quality and convert to 1.
                        map[index[0], index[1]] = 1
                    # Finally we invert the map, so 0 is free space and 1 is occupied space.
                    self.map = np.logical_not(map).astype(int)
                    self.map_meters = meters
                else:
                    self.__init__(values)
            else:
                raise ValueError("Could not extract map from .pkl file.")

        elif isinstance(map, bytearray):
            # convert from slam to IR
            # Slam maps are bytearrays that represent the SLAM Calculated map. Higher the pixel value, clearer it is.
            map = np.array(map).reshape(int(len(map) ** 0.5), int(len(map) ** 0.5))
            for index in np.argwhere(map < 200):  # First we pull down the readings below quality.
                map[index[0], index[1]] = 0
            for index in np.argwhere(map >= 200):  # Then we pull up the readings above quality and convert to 1.
                map[index[0], index[1]] = 1
            # Finally we invert the map, so 0 is free space and 1 is occupied space.
            self.map = np.logical_not(map).astype(int)

        elif isinstance(map, np.ndarray):
            # convert from rrt to IR
            # RRT Maps are numpy arrays that RRT uses to calculate a route to a point. 1 represents an obstacle.
            if max(map) > 1:
                for index in np.argwhere(map < 200):  # First we pull down the readings below quality.
                    map[index[0], index[1]] = 0
                for index in np.argwhere(map >= 200):  # Then we pull up the readings above quality and convert to 1.
                    map[index[0], index[1]] = 1
                # Finally we invert the map, so 0 is free space and 1 is occupied space.
                self.map = np.logical_not(map).astype(int)
            else:  # If we get an IR Map, we just use it.
                self.map = map

        elif isinstance(map, int):
            # generate a blank IR Map
            self.map = np.zeros((map, map), dtype=int)

        else:
            raise ValueError("Map format unidentified.")
        self.map_center = [i//2 for i in self.map.shape]
        if self.map_meters is None:
            self.map_meters = map_meters

    def update(self, map):
        self.__init__(map)

    def fromSlam(self, map):
        size = int(len(map) ** 0.5)  # get the shape for a square map (1d to 2d)
        # convert from bytearray to 2d np array, apply quality threshold, scale down to 0-1, reshape
        map = ((np.array(map) - 73) / 255).reshape(size, size).round()
        self.map = np.logical_not(map).astype(int)

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
        free = np.argwhere(self.map == 0)
        return tuple(free[np.random.randint(0, free.shape[0])])

    def __len__(self):
        return len(self.map)

    def tocv2(self, invert=True):
        map = self.map if not invert else np.logical_not(self.map)
        return cv2.cvtColor(np.rot90(map).astype(np.uint8) * 255, cv2.COLOR_GRAY2BGR)

    def drawPoint(self, img, point, r=2, c=(0, 0, 255), t=2):
        return cv2.circle(img, (point[0], self.map.shape[0]-point[1]), r, c, t)

    def drawPx(self, img, point, c=(0, 0, 255), r=1):
        for a in range(-r, r):
            for b in range(-r, r):
                img[self.map.shape[0]-(point[1]+b)][point[0]+a] = c
        return img

    def drawLine(self, img, line, c=(0, 255, 0), **kwargs):
        return cv2.line(img, (line[0][0], self.map.shape[0]-line[0][1]),
                        (line[1][0], self.map.shape[0]-line[1][1]), c, **kwargs)

    def drawLineOfDots(self, img, line, c=(0, 255, 0)):
        [self.drawLine(img, (line[i], line[i+1]), c=c, thickness=2) for i in range(len(line)-1)]

    def getValidRoute(self, n):
        return [self.getValidPoint() for _ in range(n)]

    def _posearrowext(self, pose, r):
        return pose[:2], (round(pose[0]+r*np.cos(pose[2])), round(pose[1]+r*np.sin(pose[2])))

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
                    color = np.random.randint(0, 256, 3).tolist()
                    for line in path:
                        img = self.drawLine(img, line, color)
        if pose is not None:
            if pose == "center":
                cv2.arrowedLine(img, self.map_center, tuple(pymap(self.map_center, lambda x: x-5)), (0, 0, 255), 3)
            else:
                pt1, pt2 = self._posearrowext(pose, 20)
                cv2.arrowedLine(img, pt1, pt2, (0, 0, 255), 3)
        cv2.imshow("Map", img)
        cv2.waitKey(1)

    def addPath(self, route: np.ndarray):
        try:
            if route is None:
                return
            int(route[0][0][0])  # check if we can index that deep and the value is a number
            # If that happens, we are sure it's a path
            self.paths.append(route)
        except TypeError:  # If we could not convert to an integer,
            [self.paths.append(i) for i in route]  # It means that route[0][0][0] was an array.
        except IndexError:  # If the probe was not successful, it's invalid.
            print("Empty or Invalid path provided.")

    def collision_free(self, a, b) -> bool:
        # Bresenham's line algorithm
        y0, x0 = int(a[0]), int(a[1])
        y1, x1 = int(b[0]), int(b[1])
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy

        while True:
            if x0 < 0 or x0 >= self.map.shape[0] or y0 < 0 or y0 >= self.map.shape[1]:
                break  # Out of bounds
            if self.map[y0, x0] != 0:  # Note the y0, x0 order for numpy arrays
                return False
            if x0 == x1 and y0 == y1:
                return True
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
        return False  # Return False if out of bounds


class SLAM:
    def __init__(self, lidar=None, map_handle=None, update_map=1):
        self.map = map_handle if map_handle else Map(800)
        if lidar is None:
            from breezyslam.sensors import RPLidarA1
            lidar = RPLidarA1()
        self.mapbytes = self.map.toSlam()
        self.ShouldUpdate = update_map
        self.map_size = len(self.map)  # Ensure it's actually an integer, might've been calculated.
        self.pose = (0, 0, 0)
        # Slam Preparation
        self.ratio = self.map.map_meters*1000 / self.map_size  # For converting MM to px.
        self.slam = RMHC_SLAM(lidar, self.map_size, self.map.map_meters)
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
        return (x, y), (int(x + size * np.cos(r)), int(y + size * np.sin(r)))


@dataclass
class Node:
    x: float
    y: float
    parent: 'Node' = None

    def __getitem__(self, item):
        return (self.x, self.y)[item]


class RRT:
    def __init__(self, map, step_size=25, iterations=1000):
        self.map = map
        self.step_size = step_size
        self.iterations = iterations

    def plan(self, start, end):
        start, end = np.array(start), np.array(end)
        # If you can go straight to the goal, do it
        if self.map.collision_free(start, end):
            return [[start.tolist(), end.tolist()]]

        start_node = Node(*start)
        goal_node = Node(*end)

        # Set up 2 trees to expand
        tree_a = [start_node]
        tree_b = [goal_node]

        for _ in range(self.iterations):
            # Pick a random point in the map to explore
            # Oportunity: Reduce the randomness, to make certain-er paths
            # Maybe reduce the options to an ellipse between both points
            random_point = Node(np.random.uniform(0, self.map.map.shape[0]), np.random.uniform(0, self.map.map.shape[1]))

            # Extend tree A towards the random point
            nearest_node_a = self._nearest(tree_a, random_point)
            new_node_a = self._steer(nearest_node_a, random_point)
            if new_node_a:
                tree_a.append(new_node_a)

                # Try to connect new node in tree A to nearest node in tree B
                nearest_node_b = self._nearest(tree_b, new_node_a)
                new_node_b = self._steer(nearest_node_b, new_node_a)
                while new_node_b:
                    tree_b.append(new_node_b)
                    if new_node_a.x == new_node_b.x and new_node_a.y == new_node_b.y:  # Trees are connected
                        # Path found
                        # Combine and reverse path from goal to start
                        out = self._rearrange(self.extract_path(new_node_a) + self.extract_path(new_node_b)[::-1], start, end)
                        # Here goes all the line postprocessing fluff

                        if out[-1][1] != end.tolist():
                            out[-1] = out[-1][::-1]
                        return out

                    nearest_node_b = self._nearest(tree_b, new_node_a)
                    new_node_b = self._steer(nearest_node_b, new_node_a)

            # Swap trees
            tree_a, tree_b = tree_b, tree_a

    def _steer(self, from_node, to_node):
        # Steering is picking a new step rate for each direction (so basically an angle abstracted into sine and cosine)
        direction = np.array([to_node.x - from_node.x, to_node.y - from_node.y])  # get the distance
        norm = np.linalg.norm(direction)  # hypotenuse
        if norm == 0:
            return None  # from_node and to_node are the same

        # Get the direction in interval I = [-1, 1], and make a step via the step size or the norm, whichever is smaller.
        direction = direction / norm * min(self.step_size, norm)

        # Step towards the desired direction.
        new_node = Node(from_node.x + direction[0], from_node.y + direction[1], parent=from_node)
        if self.map.collision_free(from_node, new_node):
            return new_node
        return None

    @staticmethod
    def _nearest(nodes, target):
        # check the closest node to the target
        distances = [np.hypot(node.x - target.x, node.y - target.y) for node in nodes]
        nearest_index = np.argmin(distances)
        return nodes[nearest_index]

    def _rearrange(self, path, start, end):
        # Check if the path is inverse.

        # First, if the end point is at the start
        if np.argwhere(np.all(path[0] == end, axis=-1)).size != 0:
            path.reverse()

        # Second, if the start point is at the end
        elif np.argwhere(np.all(path[-1] == start, axis=-1)).size != 0:
            raise ValueError(f"Inverted at second check.")

        # This is a continuity checker, i think?
        # Yes, this loop stops when i is a broken bond.
        last = path[0][1]
        for i in range(1, len(path) - 1):
            segment = path[i]
            if last != segment[0]:
                break
            last = segment[1]
        else:  # The else runs when the for loop completes.
            return path

        a, b = path[:i], path[i:]  # Split the path into the convergence of both trees
        b = np.fliplr(b).tolist()  # flip the second tree
        out = a + b  # Join them back properly
        return self.restitch(out)
        # This is a continuity checker. It tries to patch holes in the path by connecting disjointed segments.

    @staticmethod
    def calculate_slope(p1, p2):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return dy / dx if dx != 0 else None  # Handle vertical lines

    @staticmethod
    def slopes_are_close(slope1, slope2, tolerance=0.1):
        # not necesarily slopes, just comparing two numbers
        if slope1 is None and slope2 is None:
            return True
        if slope1 is None or slope2 is None:
            return False
        return abs(slope1 - slope2) <= tolerance

    def extract_path(self, end_node, slope_tolerance=1e-1):
        # This traverses the tree and extracts the xy location of each node.
        # After that, it simplifies the path by removing unnecessary points, and making line segments.

        points = []
        current_node = end_node
        while current_node is not None:
            points.append([current_node.x, current_node.y])
            current_node = current_node.parent

        points = np.array(points[::-1])  # Reverse to start from the beginning of the path

        if len(points) < 2:
            return []
        # This is the simplifier. It takes a list of points and returns a list of segments.
        segments = []
        current_slope = self.calculate_slope(points[0], points[1])
        start_point = points[0]

        for i in range(1, len(points)):
            new_slope = self.calculate_slope(points[i - 1], points[i]) if i < len(points) - 1 else None
            if not self.slopes_are_close(current_slope, new_slope, slope_tolerance) or i == len(points) - 1:
                # Before finalizing the segment, check for collisions along the proposed segment
                if self.map.collision_free(start_point, points[i]):
                    # If no collision, finalize the segment
                    segments.append([start_point.astype(int).tolist(), points[i].astype(int).tolist()])
                    start_point = points[i]
                    current_slope = new_slope
                else:
                    # If there's a collision, break the segment at the last collision-free point
                    # and start a new segment from there
                    segments.append([start_point.astype(int).tolist(), points[i - 1].astype(int).tolist()])
                    start_point = points[i - 1]
                    current_slope = self.calculate_slope(points[i - 1], points[i])

        return segments

    # Path Postprocessing methods
    # Check this, it is probably very broken.
    def ecd_shortening(self, path, start, end):
        # This function clips the ends of the path to the nearest collision-free point.
        if len(path) == 1:
            return path
        print(f"Shortening path...")
        furthest_start = 0
        soonest_end = None
        map = self.map

        # Find the closest indices where the start and end points can be connected to the path directly
        for i, line in enumerate(path):
            if map.collision_free(line[0], start):
                furthest_start = i
            if soonest_end is None and map.collision_free(line[0], end):
                soonest_end = i

        new = [[start.tolist(), path[furthest_start][0]]] + path[furthest_start:soonest_end]
        if path[soonest_end][0] == end.tolist():
            print("End path will repeat")
            print(new + [[path[soonest_end][0], end.tolist()]])
            new.append([path[soonest_end - 1][1], end.tolist()])
            print(new)
        else:
            new += [[path[soonest_end][0], end.tolist()]]
        # Replace all previous segments for the shortened segments.
        print(f"Final shortening: {new}")
        return new

    @staticmethod
    def restitch_end(path):  # This line end stitching happens when the final segment is the end point repeated.
        if len(path) == 1:
            return path
        # This can lead to a serious mistake.
        end, prev = path[-1], path[-2]
        if prev[1] != end[0]:
            print(f"Stitching end: {path}")
            return path[:-1] + [[prev[1], end[0]]] + [path[-1]]
        return path

    @staticmethod
    def check_continuity(path):
        prev = path[0][1]
        for i in range(1, len(path)):
            segment = path[i]
            if prev != segment[0]:
                return False
            prev = segment[1]
        return True

    @staticmethod
    def restitch(path):
        out = path
        while True:
            last = out[0][1]  # Get first segment
            for i in range(1, len(out)):  # For every segment
                segment = out[i]  # get the segment
                if last != segment[0]:  # If the last point of the last segment is not the first point of this segment
                    a, b = out[:i], out[i:]
                    out = a + [[a[-1][1], b[0][0]]] + b  # Make a bridge between both segments
                    break  # Try the continuity check again
                last = segment[1]  # If not, move on to the next segment
            else:
                return out  # If the continuity check passes, return the path

    @staticmethod
    def getEndPoints(lines):
        lines = np.array(lines)
        return np.append([lines[0][0]], lines[:, 1], 0)

    @staticmethod
    def endpointsToPath(endpoints):
        return np.array([[endpoints[i], endpoints[i + 1]] for i in range(len(endpoints) - 1)])

    @staticmethod
    def isValidPath(path):
        return (path is not None) and len(path) > 0