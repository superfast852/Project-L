import matplotlib.pyplot as plt
from Robots.RM_HAL import RP_A1, RPLidarException, np
from extensions.ICP import iterative_closest_point, transform_to_pose
from extensions.NavStack import Map, SLAM
from extensions.logs import logging

logger = logging.getLogger(__name__)


class AnimatedWindow:
    def __init__(self):
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.gca()
        self.figid = id(self.fig)
    def scatter(self, points):
        self.ax.scatter(*zip(*points))
    def refresh(self):
        if id(plt.gcf()) != self.figid:
            raise ValueError("Window does not exist.")
        plt.draw()
        plt.pause(0.0001)
    def clear(self):
        self.ax.cla()


def run():
    lidar = RP_A1()
    window = AnimatedWindow()
    last_scan = lidar.readCartesian()
    max_side = 12/(2**0.5)*1000
    ancles = np.array([[-max_side, -max_side], [-max_side, max_side], [max_side, max_side], [max_side, -max_side], [-max_side, -max_side]]).T
    map = Map(800)
    slam = SLAM(lidar, map)
    try:
        while True:
            window.clear()
            window.ax.plot(*ancles)
            points = lidar.readCartesian()
            u = iterative_closest_point(points, last_scan)
            last_scan = points
            x, y, t = transform_to_pose(u)
            window.scatter(points)
            window.scatter(((x, y), ))
            window.refresh()
            pose = slam.update(*lidar.getScan())
            map.animate()
    except RPLidarException:
        pass
    except ValueError as e:
        if str(e) == "Window does not exist.":
            pass
        else:
            print(e)
    except KeyboardInterrupt:
        pass
    finally:
        lidar.exit()

if __name__ == '__main__':
    run()
