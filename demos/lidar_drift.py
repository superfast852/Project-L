import time
import matplotlib.pyplot as plt
from Robots.RM_HAL import RP_A1, RPLidarException, np
from extensions.ICP import iterative_closest_point, transform_to_pose, transform_points
from extensions.NavStack import Map, SLAM


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
    lidar = RP_A1(scan_type="express", threaded=False)
    window = AnimatedWindow()
    last_scan = lidar.readCartesian()
    pose = np.array([0.0, 0.0, 0.0])
    max_side = 13000
    ancles = np.array([[-max_side, -max_side], [-max_side, max_side], [max_side, max_side], [max_side, -max_side], [-max_side, -max_side]]).T
    dt = 0.1
    last_time = 0
    last_pose = np.array([0, 0, 0])
    last_v = np.array([0, 0, 0])
    v = np.array([0, 0, 0])
    try:
        while True:
            # basic frame
            curr = time.perf_counter()
            dt = curr - last_time
            last_time = curr
            window.clear()
            window.ax.plot(*ancles)

            # data processing
            points = -lidar.readCartesian()
            #u = iterative_closest_point(points, last_scan, 10000, 1e-7)

            #x, y, t = transform_to_pose(u)
            #transformed = transform_points(u, points)

            # visualization
            #window.scatter(last_scan)
            #window.scatter(transformed)
            window.scatter(points)
            window.scatter(((0, 0), ))  # see the center
            window.refresh()

            #last_pose = pose.copy()
            #pose += np.array([x, y, t])

            #v = (pose - last_pose) / dt
            #a = (v - last_v) / dt
            #last_v = v.copy()

            #if np.linalg.norm(a) > 1000:
            #    pose = last_pose
            #else:
            #    print(f"Pose: {pose.astype(float).round(2).tolist()}, Acceleration: {a.round(2).tolist(), np.linalg.norm(a)}")
            last_scan = points
    except RPLidarException:
        print("Something has gone wrong!")
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