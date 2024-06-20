# TODO: Deprecate the original map and replace entirely for TransientMap. Port RRT and the rest of the stuff over.

from __future__ import annotations
from dataclasses import dataclass
from breezyslam.algorithms import RMHC_SLAM
from pickle import dump, load
import numpy as np
import cv2
from extensions.logs import logging
from numba import njit, prange
from numba.core.errors import NumbaDeprecationWarning, NumbaPendingDeprecationWarning
import warnings

warnings.simplefilter('ignore', category=NumbaDeprecationWarning)
warnings.simplefilter('ignore', category=NumbaPendingDeprecationWarning)
nb_log = logging.getLogger('numba')
nb_log.setLevel(logging.ERROR)
logger = logging.getLogger(__name__)

def pymap(n, func):
    return map(func, n)


class Map:
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
            elif map.endswith(".pkl"):
                with open(map, "rb") as f:
                    values = load(f)
                if len(values) == 2 and (isinstance(values, tuple) or isinstance(values, list)):
                    meters, bytearr = values
                    if isinstance(bytearr, bytearray):
                        self.fromSlam(bytearr)
                        self.map_meters = meters
                    else:
                        self.__init__(values, meters)
                else:
                    self.__init__(values)
            else:
                logger.error("[NavStack/Map] Could not load map from file.")
                raise ValueError("Could not extract map from .pkl file.")

        elif isinstance(map, bytearray):
            # convert from slam to IR
            # Slam maps are bytearrays that represent the SLAM Calculated map. Higher the pixel value, clearer it is.
            self.fromSlam(map)

        elif isinstance(map, np.ndarray):
            if np.max(map) > 1:
                self.map = self.nb_transient(map.flatten()) if map.ndim == 2 else self.nb_transient(map)
            else:  # If we get an IR Map, we just use it.
                self.map = map

        elif isinstance(map, int):
            # generate a blank IR Map
            self.map = np.ones((map, map), dtype=int)*-1

        else:
            logger.error("[NavStack/Map] Map format unidentified.")
            raise ValueError("Map format unidentified.")

        self.map_center = (self.map.shape[0]//2, )*2  # These are all square maps, so no need to worry.

        if self.map_meters is None:
            self.map_meters = map_meters
        self.m2px = self.map.shape[0] / self.map_meters
        self.px2m = 1 / self.m2px
        self.collision_free((0, 0), (0, 0))

    def update(self, map):
        self.__init__(map)

    def fromSlam(self, map: bytearray):
        self.map = self.nb_transient(np.array(map)).astype(int)

    @staticmethod
    @njit(parallel=True)
    def nb_transient(map_array: np.ndarray) -> np.ndarray:
        tol = 1e-2
        len = int(map_array.size ** 0.5)
        map = (map_array.reshape(len, len) - 73) / 255
        for i in prange(map.shape[0]):
            for j in range(map.shape[1]):
                if abs(map[i, j] - (54 / 255)) < tol:
                    map[i, j] = -1

        mask = map != -1

        for i in prange(map.shape[0]):
            for j in range(map.shape[1]):
                if mask[i, j]:
                    map[i, j] = np.logical_not(round(map[i, j]))

        return map

    def toSlam(self):
        map = self.map.copy()
        map[map == -1] = 127
        map[map == 0] = 255
        map[map == 1] = 0
        return bytearray(map.flatten())

    def save(self, name=None):
        if name is None:
            from datetime import datetime
            datetime = datetime.today()
            name = f"{datetime.day}-{datetime.month}-{datetime.year} {datetime.hour}:{datetime.minute}.pkl"
            del datetime
        with open(name, "wb") as f:
            dump(self.map, f)
        logging.info(f"[NavStack/Map] Saved Map as {name}!")

    def isValidPoint(self, point, unknown=False):
        return self.map[point[1], point[0]] == 0 if not unknown else self.map[point[1], point[0]] != 1

    def getValidPoint(self, unknown=False) -> tuple:
        free = np.argwhere(self.map == 0) if not unknown else np.argwhere(self.map != 1)
        return tuple(free[np.random.randint(0, free.shape[0])][::-1])  # flip to get as xy

    def __len__(self):
        return len(self.map)

    def __getitem__(self, item):
        if len(item) != 2:
            logger.error("[NavStack/Map] Index of the map must be a point (X, Y).")
            raise IndexError("Index of the map must be a point (X, Y).")
        return self.map[item[1], item[0]]

    def __repr__(self):
        return f"TransientMap({self.map.shape}, {self.map_meters})"

    def tocv2(self, invert=1, img=None):  # oh boy.
        map = self.map.copy() if img is None else img
        if invert:
            mask = map != -1
            map[mask] = np.logical_not(map[mask])*255
            map[map == -1] = 127
        else:
            map *= 255
            map[map == -255] = 127
        return cv2.cvtColor(map.astype(np.uint8), cv2.COLOR_GRAY2BGR)

    def drawPoint(self, img, point, r=2, c=(0, 0, 255), t=2):
        return cv2.circle(img, point, r, c, t)

    def drawPx(self, img, point, c=(0, 0, 255), r=1):
        for a in range(-r, r):
            for b in range(-r, r):
                img[point[1]+a, point[0]+b] = c
        return img

    def drawLine(self, img, line, c=(0, 255, 0), **kwargs):
        return cv2.line(img, *line, c, **kwargs)

    def drawLineOfDots(self, img, line, c=(0, 255, 0)):
        [self.drawLine(img, (line[i], line[i+1]), c=c, thickness=2) for i in range(len(line)-1)]

    def getValidRoute(self, n, unknown=False):
        return [self.getValidPoint(unknown) for _ in range(n)]

    def _posearrowext(self, pose, r):
        x = r * np.cos(pose[2])
        y = r * np.sin(pose[2])
        return (round(pose[0] - x), round(pose[1] - y)), (round(pose[0] + x), round(pose[1] + y))

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
            logging.error("[NavStack/Map] Empty or Invalid path provided.")
        return

    def animate(self, img=None, pose=None, drawLines=True, arrowLength=20, thickness=5, show="Map"):
        # Pose is expected to be 2 coordinates, which represent a center and a point along a circle.
        if img is None:
            img = self.tocv2()
        if drawLines:
            for path in self.paths:
                try:
                    path = path.tolist()
                except AttributeError:  # Means it's already a list.
                    pass
                if path:
                    cv2.circle(img, path[0][0], 5, (127, 127, 127), -1)
                    cv2.circle(img, path[-1][1], 5, (0, 255, 0), -1)
                    for line in path:
                        cv2.line(img, *line, [211, 85, 186])
        if pose is not None:
            if pose == "center":
                cv2.arrowedLine(img, self.map_center, tuple(pymap(self.map_center, lambda x: x-5)), (0, 0, 255), 3)
            else:
                pt1, pt2 = self._posearrowext(pose, arrowLength/2)
                cv2.arrowedLine(img, pt1, pt2, (0, 0, 255), thickness)
        if show:
            cv2.imshow(show, img)
            cv2.waitKey(1)
        else:
            return img

    def binary(self, val=1):
        map = self.map.copy()
        map[map == -1] = val
        return val

    def collision_free(self, a, b) -> bool:
        return self.cf_wrap(self.map, (a[0], a[1]), (b[0], b[1]))

    @staticmethod
    @njit
    def cf_wrap(map, a, b) -> bool:
        x1, y1, x2, y2 = int(a[0]), int(a[1]), int(b[0]), int(b[1])
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        issteep = abs(y2 - y1) > abs(x2 - x1)
        if issteep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
        deltax = x2 - x1
        deltay = abs(y2 - y1)
        error = int(deltax / 2)
        y = y1
        if y1 < y2:
            ystep = 1
        else:
            ystep = -1
        for x in range(x1, x2 + 1):
            if issteep:
                point = (y, x)
            else:
                point = (x, y)
            if (point[0] < 0 or point[1] < 0) or (point[0] >= map.shape[0] or point[1] >= map.shape[1]):
                return False
            if map[point[1], point[0]] == 1:  # there is an obstacle
                return False
            error -= deltay
            if error < 0:
                y += ystep
                error += deltax
        return True



class SLAM:
    def __init__(self, lidar=None, map_handle=None, update_map=1):
        self.map = map_handle if map_handle else Map(800)
        if lidar is None:
            from breezyslam.sensors import RPLidarA1
            lidar = RPLidarA1()
        self.mapbytes = self.map.toSlam()
        self.ShouldUpdate = update_map
        self.pose = (0, 0, 0)
        # Slam Preparation
        self.ratio = self.map.px2m*1000

        self.slam = RMHC_SLAM(lidar, len(self.map), self.map.map_meters, map_quality = 50)
        self.slam.setmap(self.mapbytes)

    def update(self, distances, angles, odometry=None):
        if angles is None:
            logger.error("[NavStack/SLAM] Angles are required for SLAM. The array method has been deprecated.")
            raise ValueError("Angles are required for SLAM. The array method has been deprecated.")
        self.slam.update(distances, odometry, angles, self.ShouldUpdate)
        # keep in mind that SLAM basically remains running on its own map. So no need to worry about data loss
        if self.ShouldUpdate:
            self.slam.getmap(self.mapbytes)  # as seen here, SLAMOps run on self.mapbytes exclusively.
            self.map.fromSlam(self.mapbytes)  # then, it gets exported to self.map
        self.pose = self.slam.getpos()
        return self.pose2px(self.pose)  # SLAM

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
    def __init__(self, map: Map, step_size=25, iterations=1000, inflate=0.0, debug=False):
        self.map = map
        self.step_size = step_size
        self.iterations = iterations
        self.debug = debug
        self.inflate = inflate
        self.inflated = Map(self.map.map)
        self.map_shape = self.map.map.shape

    def plan(self, start, end):
        start, end = np.array(start), np.array(end)
        self.map_shape = self.map.map.shape
        if self.inflate > 0:
            self.inflated.map = self.gen_binary_cost(self.inflate, start=start, stop=end)
        if self.debug:
            self.img = self.map.tocv2()
            cv2.circle(self.img, start, 5, (0, 255, 0), -1)
            cv2.circle(self.img, end, 5, (0, 255, 0), -1)
        # If you can go straight to the goal, do it
        if self.inflated.collision_free(start, end):
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
            random_point = Node(np.random.uniform(0, self.map_shape[0]), np.random.uniform(0, self.map_shape[1]))
            # Extend tree A towards the random point
            nearest_node_a = self._nearest(tree_a, random_point)
            new_node_a = self._steer(nearest_node_a, random_point)
            if new_node_a:
                tree_a.append(new_node_a)
                if self.debug:
                    cv2.circle(self.img, [int(new_node_a[0]), int(new_node_a[1])], 5, (0, 0, 255), -1)
                # Try to connect new node in tree A to nearest node in tree B
                nearest_node_b = self._nearest(tree_b, new_node_a)
                new_node_b = self._steer(nearest_node_b, new_node_a)
                while new_node_b:
                    tree_b.append(new_node_b)
                    if self.debug:
                        cv2.circle(self.img, [int(new_node_b[0]), int(new_node_b[1])], 5, (255, 0, 0), -1)
                    if new_node_a.x == new_node_b.x and new_node_a.y == new_node_b.y:  # Trees are connected
                        if self.debug:
                            cv2.circle(self.img, [int(new_node_a[0]), int(new_node_a[1])], 5, (255, 0, 255), -1)
                            self.map.animate(self.img)
                            cv2.waitKey(1000)
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
            if self.debug:
                self.map.animate(self.img)
                cv2.waitKey(100)

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

    def _rearrange(self, path, start, end):
        # Check if the path is inverse.

        # First, if the end point is at the start
        if np.argwhere(np.all(path[0] == end, axis=-1)).size != 0:
            path.reverse()

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
    # TODO: Check this, it is probably very broken.
    def ecd_shortening(self, path, start, end):
        # This function clips the ends of the path to the nearest collision-free point.
        if len(path) == 1:
            return path
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
            new.append([path[soonest_end - 1][1], end.tolist()])
        else:
            new += [[path[soonest_end][0], end.tolist()]]
        # Replace all previous segments for the shortened segments.
        return new

    def gen_binary_cost(self, inflation_r=8.5, start=None, stop=None):
        '''
        Inflates the obstacles in the map to create a binary cost map. This is useful for path planning.
        :param og: The occupancy grid (map.map)
        :param inflation_r: The inflation radius around the obstacles. A higher radius mean more centered paths, but less likely to generate a path. A smaller one means shorter paths, but closer to walls.
        :param start: The start point of the path
        :param stop: The end point of the path
        :return: Inflated binary cost map
        '''

        cut = round(inflation_r)
        og_points = np.argwhere(self.map.map == -1)  # Extract the location of the unknowns
        sample = self.map.map.copy()
        sample[sample == -1] = 0
        sample = np.logical_not(sample).astype(np.uint8)

        edt = cv2.distanceTransform(sample, cv2.DIST_L2, 3)
        sample[edt < inflation_r] = 0
        out = np.logical_not(sample)
        if start is not None:  # If start is not None, then end mustn't be either.
            out[start[1] - cut:start[1] + cut, start[0] - cut:start[0] + cut] = 0
            out[stop[1] - cut:stop[1] + cut, stop[0] - cut:stop[0] + cut] = 0

        out = out.astype(int)
        out[og_points[:, 1], og_points[:, 0]] = -1
        return out

    @staticmethod
    def restitch_end(path):  # This line end stitching happens when the final segment is the end point repeated.
        if len(path) == 1:
            return path
        # This can lead to a serious mistake.
        end, prev = path[-1], path[-2]
        if prev[1] != end[0]:
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

    @staticmethod
    def _nearest(nodes, target):
        # check the closest node to the target
        distances = [np.hypot(node.x - target.x, node.y - target.y) for node in nodes]
        nearest_index = np.argmin(distances)
        return nodes[nearest_index]


if __name__ == '__main__':
    map = Map("../Resources/TransientMap_test_scans.pkl")
    rrt = RRT(map, inflate=8.5)
    inflated = rrt.gen_binary_cost()
    img = map.tocv2(invert=True, img=inflated)
    while True:
        map.paths = []
        start = map.getValidPoint()
        end = map.getValidPoint()
        path = rrt.plan(start, end)
        if rrt.isValidPath(path):
            map.addPath(path)
            map.animate(img)
            cv2.waitKey(1000)
        else:
            print("No path found.")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            break

