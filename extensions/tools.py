import numpy as np
from serial.tools.list_ports import comports
from numba import njit
from extensions.logs import logging
from numba.core.errors import NumbaDeprecationWarning, NumbaPendingDeprecationWarning
import warnings

warnings.simplefilter('ignore', category=NumbaDeprecationWarning)
warnings.simplefilter('ignore', category=NumbaPendingDeprecationWarning)
numba_logger = logging.getLogger('numba')
numba_logger.setLevel(logging.ERROR)
logger = logging.getLogger(__name__)



# Think I should clarify that this is absolutely fucking disgusting and nobody should ever do this at all ever.
def exceptionless_exec(f):
    try:
        f()
    except Exception as e:
        logger.warning(f"[exceptionless_exec] {e.args} \n\tContext: {e.__context__}\n\tCause: {e.__cause__}\n\tTraceback: {e.__traceback__}")


def find_port_by_vid_pid(vid, pid):
    ports = list(comports())

    for port in ports:
        if port.vid == vid and port.pid == pid:
            return port.device
    return None


# IDEA: Bézier curve as smoothing function??? wink wink
def smoothSpeed(start, stop, lapse=100):
    def smooth(stamp):
        t = stamp/lapse
        if t < 0.5:
            res = 4 * t * t * t
        else:
            p = 2 * t - 2
            res = 0.5 * p * p * p + 1
        return stop*res + start*(1-res)

    return smooth


def getAngle(x, y):
    angle = round(np.arctan2(y, x), 6)
    if angle < 0:
        angle += 2* np.pi
    return angle


def inTolerance(a, b, tol=1):
    return abs(a - b) <= tol


def limit(x, low, high):
    return min(high, max(x, low))


# ---- Bezier Curves :) ----
def quad_bezier(start, stop, ctrl):
    start, stop, ctrl = np.array([start, stop, ctrl])
    P = lambda t: (1-t)**2 * start + 2 * t * (1-t) * ctrl + t**2 * stop
    return np.array([P(t) for t in np.linspace(0, 1, 50)])


@njit
def line2dots(a, b):
    x1, y1, x2, y2 = a[0], a[1], b[0], b[1]
    points = []
    issteep = abs(y2-y1) > abs(x2-x1)
    if issteep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    rev = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        rev = True
    deltax = x2 - x1
    deltay = abs(y2-y1)
    error = int(deltax / 2)
    y = y1
    ystep = None
    if y1 < y2:
        ystep = 1
    else:
        ystep = -1
    for x in range(x1, x2 + 1):
        if issteep:
            points.append((y, x))
        else:
            points.append((x, y))
        error -= deltay
        if error < 0:
            y += ystep
            error += deltax
    # Reverse the list if the coordinates were reversed
    if rev:
        points.reverse()
    return points


def avg(l):
    return sum(l)/len(l)


@njit
def ecd(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    a = a.astype(np.float64)
    b = b.astype(np.float64)
    return np.linalg.norm(a-b)


# Detection stuff
def getLabelsFromTxt(path="coco-lbl.txt", verbose=True):
    with open(path, "r") as lbls:  # Open txt file
        a = lbls.read()  # Read txt file
        b = a.split('\n')  # Split by every line into list
        return b

coco = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
         'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
         'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
         'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
         'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
         'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
         'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
         'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
         'hair drier', 'toothbrush']


def gen_sines(n_pts, dev, x_shift, rot, trans):

    """
    Generates a sine wave and creates a noisy and shifted copy for ICP testing.

    :param n_pts: Number of points to use.
    :param dev: Standard deviation of points (noise). Tends to be from 0 to 1.
    :param x_shift: Shift the noised sine wave (in radians).
    :param rot: Rotation of the noised sine wave (in radians).
    :param trans: Translation of the noised sine wave.
    :return: The pure sine wave and the noised sine wave.
    """

    x = np.linspace(0, 6.28, n_pts)
    y = np.sin(x)  # Pure target
    # Add the noise to the data
    y_shift = np.sin(x + x_shift)
    if len(y_shift) > 1000:
        y_shift = y_shift[:1000]
    y_noised = y_shift + np.random.normal(0, dev, n_pts)

    def morph_graph(points, translation, theta):
        # So first we translate the function appropriately. We expect points to be shape (N, 2)
        # where N is the number of points in the graph

        # Simple to apply the translation.
        points = points + translation
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                    [np.sin(theta), np.cos(theta)]])

        return np.dot(points, rotation_matrix)

    return np.array([x, y]).T, morph_graph(np.array([x, y_noised]).T, trans, rot)


def gen_atsushi(nPoint = 1000, fieldLength=100.0, motion=(0.5, 2.0, np.deg2rad(-10.0))):
    px = (np.random.rand(nPoint) - 0.5) * fieldLength
    py = (np.random.rand(nPoint) - 0.5) * fieldLength
    previous_points = np.vstack((px, py))

    # current points
    cx = [np.cos(motion[2]) * x - np.sin(motion[2]) * y + motion[0]
          for (x, y) in zip(px, py)]
    cy = [np.sin(motion[2]) * x + np.cos(motion[2]) * y + motion[1]
          for (x, y) in zip(px, py)]
    current_points = np.vstack((cx, cy))
    return previous_points.T, current_points.T


@njit
def pol2cart(scans: np.ndarray):
    # If i'm not mistaken, they're shaped [[r, theta], ...]
    r, theta = scans.T
    return r * np.cos(theta), r * np.sin(theta)


points = np.array([[5, i] for i in np.linspace(0, 6.28, 2)])
pol2cart(points)
ecd(np.array([1, 1]), np.array([2, 2]))
line2dots([0, 0], [5, 5])
