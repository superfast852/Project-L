# TODO: Test the coordinate system. Note that SLAM is the one to determine that, since it's who generates the map.
# Remember that this module works exclusively on meters, so aim for that.
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from pickle import load, dump
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1


class Map:
    paths = []

    def __init__(self, map="random", map_meters=35):
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
                    if isinstance(values[1], bytearray):
                        self.fromSlam(values[1])

                    elif isinstance(values[1], np.ndarray):
                        if values[1].shape[0] == values[1].shape[1]:  # check if it's square.
                            self.map = values[1]

                    self.map_meters = values[0]
                else:
                    self.__init__(values)
            else:
                raise ValueError("Could not extract map from .pkl file.")

        elif isinstance(map, bytearray):
            self.fromSlam(map)

        elif isinstance(map, np.ndarray):
            # convert from rrt to IR
            # RRT Maps are numpy arrays that RRT uses to calculate a route to a point. 1 represents an obstacle.
            if max(map) > 1 :
                for index in np.argwhere(map < 200):  # First we pull down the readings below quality.
                    map[index[0], index[1]] = 0
                for index in np.argwhere(map >= 200):  # Then we pull up the readings above quality and convert to 1.
                    map[index[0], index[1]] = 1
                # Finally we invert the map, so 0 is free space and 1 is occupied space.
                self.map = np.logical_not(map).astype(int)
            else:  # If we get an IR Map, we just use it.
                self.map = map

        elif isinstance(map, int):
            # generate a blank IR Map. All points in this map are undiscovered.
            self.map = np.zeros((map, map), dtype=int)-1

        else:
            raise ValueError("Map format unidentified.")
        self.map_center = [i//2 for i in self.map.shape]
        if self.map_meters is None:
            self.map_meters = map_meters

        self._topx = len(self.map) / self.map_meters
        self._tom = self.map_meters / len(self.map)

    def fromSlam(self, bytes: bytearray):
        # Reshape the bytearray to match the map shape
        size = int(len(bytes) ** 0.5)
        map = np.array(bytes).reshape((size, size))
        # Apply the gates
        # Map values greater or equal to 150 to 0 (free space)
        # Map values less or equal to 100 to 1 (obstacle)
        # Map values equal to 127 to -1 (undiscovered)
        map_converted = np.zeros_like(map).astype(int)
        map_converted[(map>120) & (map<150)] = -1
        map_converted[map <= 120] = 1
        map_converted[map >= 150] = 0
        self.map = map_converted

    def toSlam(self):
        pass
        original_map = np.full(self.map.shape, 127, dtype=np.uint8)

        # Map the value 0 (free space) back to 150 or above
        original_map[self.map == 0] = 255

        # Map the value 1 (obstacle) back to 100 or below
        original_map[self.map == 1] = 0

        # Convert to bytearray before returning
        return bytearray(original_map.tobytes())

    def save(self, name=None):
        if name is None:
            from datetime import datetime
            datetime = datetime.today()
            name = f"{datetime.day}-{datetime.month}-{datetime.year} {datetime.hour}:{datetime.minute}.pkl"
            del datetime

        with open(name, "wb") as f:
            dump((self.map_meters, self.map), f)
        print(f"[INFO] Saved Map as {name}!")

    def isValidPoint(self, point):
        return not self.map[point[0], point[1]]

    def getValidPoint(self) -> tuple:
        free = np.argwhere(self.map == 0)
        return tuple(free[np.random.randint(0, free.shape[0])])

    def __len__(self):
        return len(self.map)

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

    # The next two functions are intended for RRT

    def px2m(self, px):
        return px[0]*self._tom, px[1]*self._tom

    def m2px(self, m):
        if len(m) == 2:
            return int(m[0]*self._topx), int(m[1]*self._topx)
        else:
            return int(m[0]*self._topx), int(m[1]*self._topx), m[2]

    def animate(self):
        pass


class SLAM:
    # Note that the SLAM estimated position angles are in degrees.
    def __init__(self, map_handle=None, lidar=None, update_map=1):
        self.map = map_handle
        if lidar is None:
            lidar = RPLidarA1()

        self.mm2m = lambda x: (x[0] / 1000, x[1] / 1000, x[2])  # For converting MM to M.
        self.mapbytes = self.map.toSlam()
        self.ShouldUpdate = update_map
        self.pose = (0, 0, 0)

        # Slam Preparation
        self.slam = RMHC_SLAM(lidar, len(self.map), self.map.map_meters)
        self.slam.setmap(self.mapbytes)

    def update(self, distances, angles=None, odometry=None):
        # TODO: Deprecate the complete map approach. It's inefficient and can mess up the actual map.
        # You can choose what angles to feed the slam algo. That way, if the lidar's blocked, you can filter it.
        if angles is None:
            angles = list(range(360))  # we assume it's a premade map of 360 degrees (idiotic btw, dont use this.)
        elif not isinstance(angles, list):
            angles = list(angles)  # Ensure that angles is a list

        # Clipping the data to have same lengths
        if len(angles) > len(distances):
            angles = angles[:len(distances)]
        elif len(distances) > len(angles):
            distances = distances[:len(angles)]

        self.slam.update(distances, odometry, angles, self.ShouldUpdate)
        # keep in mind that SLAM basically remains running on its own map. So no need to worry about data loss

        self.slam.getmap(self.mapbytes)  # as seen here, SLAMOps run on self.mapbytes exclusively.
        self.map.fromSlam(self.mapbytes)  # then, it gets exported to self.map
        self.pose = self.mm2m(self.slam.getpos())
        return self.pose


@dataclass
class Node:
    x: float
    y: float
    parent: 'Node' = None

    def __getitem__(self, item):
        return (self.x, self.y)[item]


class RRT:  # TODO: Still need to convert this to meters n stuff
    # TODO: Adapt the inputs, and the path output to be in meters.
    # For this, use the inputParser, and the map's px2m and m2px functions.
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
            random_point = Node(np.random.uniform(0, self.map.map.shape[0]),
                                np.random.uniform(0, self.map.map.shape[1]))

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
                        out = self._rearrange(self.extract_path(new_node_a) + self.extract_path(new_node_b)[::-1],
                                              start, end)
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

    # WARNING: This function takes exclusively meters as input. Avoid it if you're using pixels.
    def inputParser(self, start: list | tuple | np.ndarray, goal: list | tuple | np.ndarray):
        if (isinstance(start, list) or isinstance(start, tuple)) and (isinstance(goal, list) or isinstance(goal, tuple)):
            start = np.array(start[:2])
            goal = np.array(goal[:2])
        elif isinstance(start, np.ndarray) and isinstance(goal, np.ndarray):
            if len(start) == 2 and len(goal) == 2:
                pass
            else:
                raise ValueError("[ERROR] Input arrays have a wrong size.")

        else:
            raise TypeError("Input must be a list, tuple or numpy array.")

        if not self.map.isValidPoint(goal):
            print(f"[ERROR] Planner: Goal at {goal} is an Obstacle!")
            return None
        elif not self.map.isValidPoint(start):
            print(f"[ERROR] Planner: Start at {start} is an Obstacle!")
            return None

        return self.map.m2px(start), self.map.m2px(goal)