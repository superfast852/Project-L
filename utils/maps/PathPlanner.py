import rrtplanner
from rrtplanner import RRTStarInformed, random_point_og
from rrtplanner.plots import plot_og, plot_start_goal, plot_rrt_lines, plot_path
from matplotlib.pyplot import subplots, show
import numpy as np


class Planner:
    def __init__(self, map, n=2000):
        self.og = map
        self.planner = RRTStarInformed(self.og, n, 64, 12)
        self.tree = None

    def plan(self, start, goal):
        self.tree, goal_vertex = self.planner.plan(start, goal)
        return self.planner.vertices_as_ndarray(self.tree, self.planner.route2gv(self.tree, goal_vertex)).tolist()

    def plot(self, path, start, goal):
        fig, ax = subplots(1, 1)
        plot_og(ax, self.og)
        plot_start_goal(ax, start, goal)
        plot_path(ax, path)
        show()

    def PATH2SLAM(self):
        map = np.logical_not(self.og).astype(int)
        for index in np.argwhere(map == 1):
            map[index[0]][index[1]] = 255
        return bytearray(map.flatten().tolist())

    @staticmethod
    def lines2points(lines):
        return [line[1] for line in lines]

def pose2px(pose, map_size, map_size_meters):
    ratio = map_size_meters*1000 / map_size
    return np.array([int(pose[0] / ratio), int(pose[1] / ratio)])


def px2pose(px, map_size, map_size_meters):
    ratio = map_size_meters / map_size
    return np.array([px[0] * ratio, px[1] * ratio])


if __name__ == "__main__":
    from SLAM import SLAM, generate_random_map
    from LD06 import LD06
    from matplotlib.pyplot import imshow, show
    map = generate_random_map(300)
    slam = SLAM(LD06(), map, 30, graph=1)
    map = slam.toPath()
    planner = Planner(map, 500)
    imshow(slam.MapToXY())
    show()

    start = pose2px(slam.update(), slam.map_size, slam.map_size_meters)
    print(start)
    goal = random_point_og(map)
    lines = planner.plan(start, goal)
    print(planner.lines2points(lines))
    planner.plot(lines, start, goal)

