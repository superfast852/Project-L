# TODO: Implement matplotlib animation. (https://pythonprogramming.net/live-graphs-matplotlib-tutorial/)
# SLAM
from breezyslam.algorithms import RMHC_SLAM

# Path Planning
from extensions.rrt_backend import RRTStarInformed, random_point_og

# Other utilities
from _pickle import dump
from numpy import array, argwhere, logical_not, ndarray, max, load


class Map:
    def __init__(self, map):
        # The IR Map is just the RRT Map format.

        if map == "random":
            # Get a randomly generated map for testing.
            try:
                self.map = load("./map.npy")  # Formatted as an RRT Map.
            except FileNotFoundError:
                self.map = load("../map.npy")

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
        print(self.map[point[0], point[1]])
        return not self.map[point[0], point[1]]

    def getValidPoint(self):
        return random_point_og(self.map)

    def __len__(self):
        return len(self.map)


class SLAM:
    def __init__(self, lidar, map_handle, map_meters, update_map=1):
        self.map = map_handle
        self.mapbytes = self.map.toSlam()
        self.ShouldUpdate = update_map
        self.map_size = len(self.map)  # Ensure it's actually an integer, might've been calculated.
        print(self.map_size)

        # Slam Preparation
        self.ratio = map_meters*1000 / self.map_size  # For converting MM to px.
        self.slam = RMHC_SLAM(lidar, self.map_size, map_meters)
        self.slam.setmap(self.mapbytes)

    def update(self, distances, angles):
        # You can choose what angles to feed the slam algo. That way, if the lidar's blocked, you can filter it.
        if angles is None:
            if len(distances) == 360:
                angles = [i for i in range(360)]
            else:
                print("[ERROR] SLAM: No angles provided for SLAM update or Distances is not a 360-value map.")
                return None
        self.slam.update(distances, scan_angles_degrees=angles, should_update_map=self.ShouldUpdate)
        self.slam.getmap(self.mapbytes)
        self.map.update(self.mapbytes)
        return self.pose2px(self.slam.getpos())  # SLAM

    def pose2px(self, pose):
        return round(pose[0] / self.ratio), round(pose[1] / self.ratio), pose[2]  # theta is just rotation so it's fine

    def px2pose(self, px):
        return px[0] * self.ratio, px[1] * self.ratio, px[2] if len(px) == 3 else None


class RRT:
    def __init__(self, n=1000, obstacle_distance=64, goal_radius=12):
        self.n = n
        self.r_rewire = obstacle_distance
        self.r_goal = goal_radius

    def plan(self, start, goal, map):
        if not map.isValidPoint(goal):
            print(f"[ERROR] Planner: Goal at {goal} is an Obstacle!")
            return None  # None indicates no point could be found. We avoid computation that way.

        # Create planner with latest map.
        planner = RRTStarInformed(map.map, self.n, self.r_rewire, self.r_goal)
        # Planner asks for coords to be arrays, so we convert them. Also make sure they're 2D.
        start = array(start[0:2])
        goal = array(goal[0:2])

        tree, route = planner.plan(start, goal)  # Execute RRT* Informed.
        if route is None:
            print("[ERROR] Planner: No path was found! Try increasing iterations or decreasing rewire radius.")
            return None
        lines = planner.vertices_as_ndarray(tree, planner.route2gv(tree, route))  # Convert route to lines.
        # The lines are composed by arrays in form [[start point, end point], [start point, end point], ...]
        # So we'll just extract every end point to get the waypoints.
        # Remember, these are pixels.
        del planner, tree, route, start, goal  # Delete planner and map to free up memory.
        return lines

    @staticmethod
    def getWaypoints(lines):
        return [i[1] for i in lines]


if __name__ == "__main__":
    from matplotlib.pyplot import imshow, show

    planner = RRT()
    map = Map("random")
    path = planner.plan(map.getValidPoint(), map.getValidPoint(), map)
    print(path)
    imshow(map.map, cmap="Greys")
    show()
