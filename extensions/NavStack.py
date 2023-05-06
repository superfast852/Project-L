# SLAM
from breezyslam.algorithms import RMHC_SLAM
from roboviz import MapVisualizer

# Path Planning
from rrtplanner import RRTStarInformed
from rrtplanner.oggen import perlin_occupancygrid

# Obstacle Avoidance
from extensions.collision import check_collision, check_surroundings

# Other utilities
from _pickle import load, dump
from numpy import array, argwhere, logical_not
from copy import deepcopy


class AutoNav:
    # General Navigation class for our robot.
    # It handles SLAM and Path Planning, along with poses.
    # The standard unit of measurement is pixels. This is done for easier planner usage and avoid world-map conversion.
    # Due to this, we must convert pose to pixels. We calculate the pixel-To-MM ratio and multiply it with the pose.

    # TODO: Add Map structure check (AKA check if an imported or given map is valid.
    # TODO: Map Cleaning. Maybe apply some gaussian filter on the map to clean up the blemishes
    # TODO: Add Plotting. Deal with the plot conflict between roboviz and rrtplot
    # TODO: Check if the RRT map is compatible with the SLAM Path. If not, make a translation layer
    # TODO: Add Odometry. You know the deal.

    def __init__(self, lidar, map, map_meters, rrtIters, r_rewire=64, r_goal=12, update_map=1, graph=1, random_map=0):
        # Map Description:
        # Maps should be 2D arrays made of 0s and 255s, determining the clearance of the map.
        # 255 is free space, 0 is occupied space.

        # SLAM makes the map, so it only needs to be converted to SLAM once.
        # Planner on the other hand, needs to be converted to planner every use, therefore we make a function for it.
        # All maps are stored as numpy arrays, with pickle.
        # Together with the map we store the meters, in the order meters -> map.
        # We can also feed the size of a map to create a new one.
        # If the map is a string, we assume it is a pkl file.
        # If the map is a bytearray, we assume it is a map.
        # If the map is an int, we assume it is the size of a map.
        if random_map:
            self.map_size = map
            self.map = logical_not(perlin_occupancygrid(map, map))  # Generates inverted random planner map
            for index in argwhere(self.map == 1):
                self.map[index[0]][index[1]] = 255  # Scale it up to actual spec
            if map_meters:
                self.map_size_meters = map_meters
            else:
                raise ValueError("If random map was generated, map_meters must be fed.")
        else:
            self.map, self.map_size, self.map_size_meters = self.extract_map(map, map_meters)
        self.mapbytes = bytearray(self.map.flatten().tolist())

        self.ShouldUpdate = update_map
        self.graph = graph
        self.map_size = int(self.map_size)  # Ensure it's actually an integer, might've been calculated.

        # Slam Preparation
        self.ratio = self.map_size_meters*1000 / self.map_size  # For converting MM to px.
        self.slam = RMHC_SLAM(lidar, self.map_size, map_meters)
        self.slam.setmap(self.mapbytes)
        if self.graph:
            self.viz = MapVisualizer(self.map_size, map_meters, "SLAM")

        # Path Planning Preparation
        self.n = rrtIters
        self.r_rewire = r_rewire
        self.r_goal = r_goal

    # SLAM
    def update(self, distances, angles):
        # You can choose what angles to feed the slam algo. That way, if the lidar's blocked, you can filter it.
        self.slam.update(distances, scan_angles_degrees=angles, should_update_map=self.ShouldUpdate)
        pose = self.slam.getpos()
        self.slam.getmap(self.mapbytes)
        self.map = self.extract_map(self.mapbytes, self.map_size_meters)[0]

        if self.graph:
            if not self.viz.display(pose[0]/1000, pose[1]/1000, pose[2], self.map):
                raise KeyboardInterrupt("Map Closed")

        return self.pose2px(pose)  # SLAM

    # RRT
    def plan(self, start, goal, quality=200):
        if self.check_point_validity(goal, quality):
            print("[ERROR] Planner: Goal was Invalid! If point was valid, try reducing quality. ")
            return None  # None indicates no point could be found. We avoid computation that way.

        # Convert map to Planner map.
        # In the standard map definition, 255 is free space, 0 is occupied space.
        # In the planner map definition, 0 is free space, 1 is occupied space.
        map = deepcopy(self.map)  # We create a complete copy to prevent changing the original map.
        for index in argwhere(map < quality):  # First we pull down the readings below quality.
            map[index[0], index[1]] = 0
        for index in argwhere(map >= quality):  # Then we pull up the readings above quality and convert to 1.
            map[index[0], index[1]] = 1
        map = logical_not(map).astype(int)  # Finally we invert the map, so 0 is free space and 1 is occupied space.

        # Create planner with latest map.
        planner = RRTStarInformed(map, self.n, self.r_rewire, self.r_goal)
        # Planner asks for coords to be arrays, so we convert them.
        start = array(start)
        goal = array(goal)

        tree, route = planner.plan(start, goal)  # Execute RRT* Informed.
        if route is None:
            print("[ERROR] Planner: No path was found! Try increasing iterations or check point validity.")
            return None
        goal_vertices = planner.route2gv(tree, route)  # Convert route to vertices.
        lines = planner.vertices_as_ndarray(tree, goal_vertices)  # Convert vertices to lines.
        # The lines are composed by arrays in form [[start point, end point], [start point, end point], ...]
        # So we'll just extract every end point to get the waypoints.
        # Remember, these are pixels.

        del planner, map, goal_vertices, tree, route, start, goal  # Delete planner and map to free up memory.
        return [line[1] for line in lines]

    def check_point_validity(self, point, threshold):
        return self.map[point] >= threshold

    # Obstacle Avoidance
    @staticmethod
    def Self_Nav(scan, collision_angles, collision_threshold):
        if check_collision(scan, collision_angles, collision_threshold):
            return check_surroundings(scan, collision_threshold)
        else:
            return "Forward"

    # Tools
    def pose2px(self, pose):
        return round(pose[0] / self.ratio), round(pose[1] / self.ratio), pose[2]  # theta is just rotation so it's fine

    def px2pose(self, px):
        if len(px) == 3:
            return px[0]*self.ratio, px[1]*self.ratio, px[2]
        else:
            return px[0] * self.ratio, px[1] * self.ratio

    def SLAM2PATH(self):
        # Converts the map and related data into Planner-Compatible Data
        pass

    def PATH2SLAM(self):
        # Converts the map and related data into SLAM-Compatible Data.
        pass

    def saveMap(self, name=None):
        if name is None:
            from datetime import datetime
            datetime = datetime.today()
            name = f"{datetime.day}-{datetime.month}-{datetime.year} {datetime.hour}:{datetime.minute}.pkl"
            del datetime
        with open(name, "wb") as f:
            dump((self.map_size_meters, self.map), f)
        print(f"[INFO] Navigator: Saved Map as {name}!")

    @staticmethod
    def extract_map(map, map_meters=None):
        if isinstance(map, str):  # SLAM* Map
            #  Pkl Check
            if not map.endswith(".pkl"):
                raise TypeError("map file must be pkl")

            with open(map, "rb") as f:
                data = load(f)
                if len(data) == 2:
                    if map_meters is None:
                        map_meters = data[0]
                    data = data[1]
                map = data
                map_size = int(len(map) ** 0.5)
            print("[INFO] Navigator: Opened Pickle Map!")

        elif isinstance(map, int):
            map_size = map
            map = array(map, map)
            print("[INFO] Navigator: Created New Blank Map!")
        elif isinstance(map, bytearray):
            map_size = int(len(map) ** 0.5)
            map = array(map).reshape(map_size, map_size)
            print("[INFO] Navigator: Opened SLAM Map!")
        else:
            raise TypeError("map must be pkl, byte map or int")
        return map, map_size, map_meters

