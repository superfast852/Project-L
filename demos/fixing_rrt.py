# Error log: the path does not get stitched properly sometimes, and a bug happens where lines are generated beyond the start or end point.

from extensions.NavStack import Map
import numpy as np
import cv2
from dataclasses import dataclass
from numba import njit


@dataclass
class Node:
    x: float
    y: float
    parent: 'Node' = None

    def __getitem__(self, item):
        return (self.x, self.y)[item]

    def __call__(self, *args, **kwargs):
        return self.x, self.y


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
            self.map, self.inflated = self.inflated, self.map
        if self.debug:
            self.img = self.map.tocv2()
            cv2.circle(self.img, start, 5, (0, 255, 0), -1)
            cv2.circle(self.img, end, 5, (0, 255, 0), -1)
        # If you can go straight to the goal, do it
        if self.map.collision_free(start, end):
            self.map, self.inflated = self.inflated, self.map
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
                        l, r = self.extract_path(new_node_a), self.extract_path(new_node_b)

                        out = self.recursive_continuity(l + self.fully_reverse(r))
                        #out = self.ecd_shortening(out, start, end)
                        self.map, self.inflated = self.inflated, self.map
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

    def fully_reverse(self, path):
        path = np.array(path)[:, ::-1]
        return path.tolist()[::-1]

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
    # TODO: Check this, it is very broken.
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

    def recursive_continuity(self, path):
        for i in range(1, len(path)):
            last = path[i-1][1]
            current = path[i][0]
            if last != current:
                print("Found discontinuity!")
                path = path[:i] + [[last, current]] + path[i:]
                return self.recursive_continuity(path)
        return path

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
    @njit(fastmath=True)
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
        np_nodes = np.array([node() for node in nodes])
        distances = np.linalg.norm(np_nodes - [target.x, target.y], axis=1)
        nearest_index = np.argmin(distances)
        return nodes[nearest_index]


if __name__ == "__main__":
    map = Map("random")
    rrt = RRT(map, inflate=0)
    while True:
        map.paths = []
        start, stop = map.getValidRoute(2)
        path = rrt.plan(start, stop)
        map.addPath(path)
        img = map.tocv2(True)
        map.drawPoint(img, start, 5, (255, 0, 0))
        map.drawPoint(img, stop, 5, (0, 0, 255))

        map.animate(img)
        cv2.waitKey(1000)
