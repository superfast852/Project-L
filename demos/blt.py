from rplidar import RPLidar, RPLidarException
import matplotlib.pyplot as plt
import numpy as np
from time import sleep, perf_counter

PORT_NAME = 'COM6'
FPS = 120


class AnimatedWindow:
    def __init__(self):
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.gca()
        self.figid = id(self.fig)

    def scatter(self, points, c=(255, 0, 0)):
        self.ax.scatter(zip(*points), c=c)

    def refresh(self):
        if id(plt.gcf()) != self.figid:
            raise ValueError("Window does not exist.")

        plt.draw()
        plt.pause(0.0001)

    def clear(self):
        self.ax.cla()


def run():
    lidar = RPLidar(PORT_NAME)
    window = AnimatedWindow()
    iterator = lidar.iter_scans()
    while True:
        try:
            window.clear()
            start = perf_counter()
            scan = next(iterator)
            points = np.array([(b*np.cos(np.radians(a)), b*np.sin(np.radians(a))) for (_, a, b) in scan])
            window.scatter(points)
            window.refresh()
            deltaTime = (perf_counter()-start)
            if deltaTime < 1/FPS:
                sleep(1/FPS - deltaTime)
        except RPLidarException:
            break
    plt.show()
    lidar.stop()
    lidar.disconnect()

if __name__ == '__main__':
    run()
