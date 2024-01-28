# Essential imports
import matplotlib.pyplot as plt
import matplotlib.cm as colormap
import matplotlib.lines as mlines
import numpy as np

# This helps with Raspberry Pi
import matplotlib
from time import time, perf_counter
from dataclasses import dataclass
matplotlib.use('TkAgg')
plt.ioff()


@dataclass
class RemovableDrawing:
    drawing: object
    previous: object = None

    def __post_init__(self):  # removing the previous drawing
        if self.previous is not None:
            self.previous.remove()
            del self.previous

    def remove(self):
        self.drawing.remove()
        self.drawing = None


class WindowClosedException(Exception):
    """Exception raised when the window is closed."""

    def __init__(self, message="Window was closed unexpectedly"):
        self.message = message
        super().__init__(self.message)


class Visualizer(object):
    # Robot display params
    ROBOT_HEIGHT_M = 0.5
    ROBOT_WIDTH_M = 0.3

    def __init__(self, map_size_pixels, map_size_meters, title, show_trajectory=False, zero_angle=0,
                 robot_dims=(0.5, 0.3)):

        # Put origin in center
        self._init(map_size_pixels, map_size_meters, title, -map_size_pixels / 2, show_trajectory, zero_angle)
        self.ROBOT_WIDTH_M, self.ROBOT_HEIGHT_M = robot_dims

    def display(self, x_m, y_m, theta_deg):

        self._setPose(x_m, y_m, theta_deg)

        return self._refresh()

    def _init(self, map_size_pixels, map_size_meters, title, shift, show_trajectory=False, zero_angle=0, trail_length=100):

        # Store constants for update
        self.map_size_pixels = map_size_pixels
        self.map_scale_meters_per_pixel = map_size_meters / float(map_size_pixels)

        # Create a byte array to display the map with a color overlay
        self.bgrbytes = bytearray(map_size_pixels * map_size_pixels * 3)

        # Make a nice big (10"x10") figure
        self.fig = plt.figure(figsize=(10, 10))

        # Store Python ID of figure to detect window close
        self.figid = id(self.fig)

        self.fig.canvas.manager.set_window_title('SLAM')
        plt.title(title)

        # Use an "artist" to speed up map drawing
        self.img_artist = None

        # No vehicle to show yet
        self.vehicle = None

        # Create axes
        self.ax = self.fig.gca()
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.grid(False)

        # Hence we must relabel the axis ticks to show millimeters
        ticks = np.arange(shift, self.map_size_pixels + shift + 100, 100)
        labels = [str(self.map_scale_meters_per_pixel * tick) for tick in ticks]
        self.ax.set_xticks(ticks)
        self.ax.set_xticklabels(labels)
        self.ax.set_yticks(ticks)
        self.ax.set_yticklabels(labels)

        # Store previous position for trajectory
        self.prevpos = None
        self.showtraj = show_trajectory
        self.trajectory = []
        self.trail_length = trail_length

        # We base the axis on pixels, to support displaying the map
        self.ax.set_xlim([shift, self.map_size_pixels + shift])
        self.ax.set_ylim([shift, self.map_size_pixels + shift])

        # Set up default shift for centering at origin
        shift = -self.map_size_pixels / 2

        self.zero_angle = zero_angle
        self.start_angle = None
        self.rotate_angle = 0
        plt.show(block=False)

    def _setPose(self, x_m, y_m, theta_deg):
        '''
        Sets vehicle pose:
        X:      left/right   (m)
        Y:      forward/back (m)
        theta:  rotation (degrees)
        '''

        # If zero-angle was indicated, grab first angle to compute rotation
        if self.start_angle is None and self.zero_angle != 0:
            self.start_angle = theta_deg
            self.rotate_angle = self.zero_angle - self.start_angle

        # Rotate by computed angle, or zero if no zero-angle indicated
        d = self.rotate_angle
        a = np.radians(d)
        c = np.cos(a)
        s = np.sin(a)
        x_m, y_m = x_m * c - y_m * s, y_m * c + x_m * s

        # Use a very short arrow shaft to orient the head of the arrow
        theta_rad = np.radians(theta_deg + d)
        c = np.cos(theta_rad)
        s = np.sin(theta_rad)
        l = 0.1
        dx = l * c
        dy = l * s

        s = self.map_scale_meters_per_pixel

        self.vehicle = RemovableDrawing(self.ax.arrow(x_m / s, y_m / s,
                                     dx, dy, head_width=Visualizer.ROBOT_WIDTH_M / s,
                                     head_length=Visualizer.ROBOT_HEIGHT_M / s, fc='r', ec='r'), self.vehicle)

        # Show trajectory if indicated
        currpos = self._m2pix(x_m, y_m)
        if self.showtraj and not self.prevpos is None:
            self.trajectory.append(self.ax.add_line(mlines.Line2D((self.prevpos[0], currpos[0]),
                                                                  (self.prevpos[1], currpos[1]))))
            if len(self.trajectory) > self.trail_length:
                self.trajectory[0].remove()
                self.trajectory = self.trajectory[1:]
        self.prevpos = currpos

    def _refresh(self):
        # If we have a new figure, something went wrong (closing figure failed)
        if self.figid != id(plt.gcf()):
            raise WindowClosedException('Figure closed')

        # Redraw current objects without blocking
        plt.draw()

        # Refresh display, setting flag on window close or keyboard interrupt
        try:
            plt.pause(.0000001)  # Arbitrary pause to force redraw
            return True
        except:
            raise WindowClosedException('Figure closed')

    def _m2pix(self, x_m, y_m):

        s = self.map_scale_meters_per_pixel

        return round(x_m / s), round(y_m / s)


class HookedVisualizer(Visualizer):
    """
    A Visualizer that can be hooked into a map object to display custom maps
    Notes:
        The visualizer flips the map upside down, so the origin is in the lower left and we can use cartesian coords.
        The visualizer also assumes that the map is square, so it will not work with rectangular maps.
        We still have to figure out how to make it compatible with RRT and SLAM.
    """
    def __init__(self, map, title='MapVisualizer', show_trajectory=False, **kwargs):

        # Put origin in lower left; disallow zero-angle setting
        self.map = map
        self.pointSplit = lambda x: list(zip(*x))
        Visualizer._init(self, len(map), map.map_meters, title, 0, show_trajectory, 0, **kwargs)

    def display(self, x_m, y_m, theta_deg, custom=None):
        self._setPose(x_m, y_m, theta_deg)
        if custom is not None:
            if isinstance(custom, bytearray):
                display = np.reshape(np.frombuffer(custom, dtype=np.uint8),
                                     (self.map_size_pixels, self.map_size_pixels))
            elif isinstance(custom, np.ndarray):
                display = custom
            else:
                raise TypeError("custom display map must be bytearray or np.ndarray")
        else:
            display = self.map.map

        # Pause to allow display to refresh
        #plt.pause(.000001)

        if self.img_artist is None:
            self.img_artist = self.ax.imshow(display, cmap=colormap.gray_r)

        else:
            self.img_artist.set_data(display)
        return self._refresh()

    def Point(self, point, previous=None):
        return RemovableDrawing(self.ax.scatter((point[0]), (point[1])), previous)

    def Line(self, line, previous=None):
        return RemovableDrawing(self.ax.add_line(mlines.Line2D(*self.pointSplit(line))), previous)

    def Pixel(self, pixel):
        pass

    def dotSequence(self, dots, previous=None, **kwargs):
        return RemovableDrawing(self.ax.scatter(*self.pointSplit(dots), **kwargs), previous)

    def Path(self, path, previous=None):
        lc = matplotlib.collections.LineCollection(path, color="r", linewidth=2, zorder=10)
        self.ax.add_collection(lc)
        return RemovableDrawing(lc, previous)

    def m2px(self, x_m, y_m):
        return self._m2pix(x_m, y_m)

    def px2m(self, x_p, y_p):
        s = self.map_scale_meters_per_pixel
        return x_p * s, y_p * s


def custom():
    from extensions.NavStack import Map, RRT
    activate_rrt = False
    pose_gen = gen_movement([17.5, 17.5, 360 * np.random.randn()], 0.1)

    map = Map("random")
    if activate_rrt:
        rrt_map = Map(map.map.T)  # WE HAD TO FUCKING TRANSPOSE IT HOLY SHIT
        rrt = RRT(rrt_map, 200)
        pathRef = None

    viz = HookedVisualizer(map, "RRT", True, trail_length=50)
    line_ref = None
    goal = (400, 450)
    while True:
        # random walk algo
        pose = next(pose_gen)
        theta = np.radians(pose[2])

        #point_ref = viz.Point(goal)
        line_ref = viz.Line([viz.m2px(*pose[:2]), viz.m2px(*pose[:2] + 2 * np.array([np.cos(theta), np.sin(theta)]))],
                            line_ref)
        if activate_rrt:
            path = rrt.plan(viz.m2px(*pose[:2]), goal)
            if rrt.isValidPath(path):
                pathRef = viz.Path(path, pathRef)
        viz.display(*pose)


def gen_movement(initial_pose, speed):
    pose = np.array(initial_pose)
    while True:
        theta = np.radians(pose[2])
        pose[0] += speed * np.cos(theta)
        pose[1] += speed * np.sin(theta)
        pose[2] += 10 * np.random.randn()
        pose[2] %= 360
        yield pose


if __name__ == '__main__':
    custom()