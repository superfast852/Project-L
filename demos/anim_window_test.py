from matplotlib import pyplot as plt
import numpy as np


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

f = lambda x: np.random.normal(x, 0.1, (10, 2))
w = AnimatedWindow()
plt.show(block=False)
while True:
    w.scatter(f(0))
    w.refresh()

