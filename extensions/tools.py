import math
from os import system
from threading import Thread
import numpy as np
from serial.tools.list_ports import comports
from numba import njit


def find_port_by_vid_pid(vid, pid):
    ports = list(comports())

    for port in ports:
        if port.vid == vid and port.pid == pid:
            return port.device
    return None


class XboxController(object):
    MAX_TRIG_VAL = 1024
    MAX_JOY_VAL = 32768

    def __init__(self, deadzone=0.1):
        from evdev import InputDevice
        for i in range(50):
            try:
                self.gamepad = InputDevice(f'/dev/input/event{i}')
                if self.gamepad.name == "Xbox Wireless Controller":
                    break
            except OSError:
                continue
        else:
            raise OSError("No controller found")
        self.deadzone = deadzone
        self.found = False
        self.LJoyY = 0
        self.LJoyX = 0  # This. Also normalize joystick values
        self.RJoyY = 0
        self.RJoyX = 0
        self.LT = 0
        self.RT = 0
        self.LB = 0
        self.RB = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LJoyB = 0
        self.RJoyB = 0
        self.Back = 0
        self.Start = 0
        self.LD = 0
        self.RD = 0
        self.UD = 0
        self.DD = 0

        self._monitor_thread = Thread(target=self._monitor_controller)
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def read(self):  # return the buttons/triggers that you care about in this methode
        reads = [self.LJoyX, self.LJoyY, self.RJoyX, self.RJoyY,
                 self.RT, self.A, self.Back, self.Start]

        return [self._clean(i) for i in reads]

    def _clean(self, x):  # Filter out the inputs.
        return round(x, 3) if not self.deadzone > x > -self.deadzone else 0

    def _monitor_controller(self):
        from evdev import ecodes
        try:
            for event in self.gamepad.read_loop():
                # Axis
                if event.type == ecodes.EV_ABS:
                    if event.code == 1:
                        self.LJoyY = self._clean(-(event.value / XboxController.MAX_JOY_VAL) + 1)  # normalize between -1 and 1
                    elif event.code == 0:
                        self.LJoyX = self._clean(event.value / XboxController.MAX_JOY_VAL - 1)  # normalize between -1 and 1
                    elif event.code == 5:
                        self.RJoyY = self._clean(-(event.value / XboxController.MAX_JOY_VAL) + 1)  # normalize between -1 and 1
                    elif event.code == 2:
                        self.RJoyX = self._clean(event.value / XboxController.MAX_JOY_VAL) - 1  # normalize between -1 and 1
                    elif event.code == 10:
                        self.LT = self._clean(event.value / XboxController.MAX_TRIG_VAL)  # normalize between 0 and 1
                    elif event.code == 9:
                        self.RT = self._clean(event.value / XboxController.MAX_TRIG_VAL)  # normalize between 0 and 1
                        # DPad

                    elif event.code == 17:
                        if event.value == 1:
                            self.UD = 1
                            self.DD = 0
                        elif event.value == -1:
                            self.UD = 0
                            self.DD = 1

                    elif event.code == 16:
                        if event.value == 1:
                            self.LD = 1
                            self.RD = 0
                        elif event.value == -1:
                            self.LD = 0
                            self.RD = 1

                elif event.type == ecodes.EV_KEY:
                    # Bumpers
                    if event.code == 310:
                        self.LB = event.value
                    elif event.code == 311:
                        self.RB = event.value

                    # Face Buttons
                    elif event.code == 304:
                        self.A = event.value
                    elif event.code == 307:
                        self.X = event.value  # previously switched with X
                    elif event.code == 308:
                        self.Y = event.value  # previously switched with Y
                    elif event.code == 305:
                        self.B = event.value

                    # Joystick Buttons
                    elif event.code == 317:
                        self.LJoyB = event.value
                    elif event.code == 318:
                        self.RJoyB = event.value

                    # Menu Buttons
                    elif event.code == 158:
                        self.Back = event.value
                    elif event.code == 315:
                        self.Start = event.value

        except Exception as e:
            print(e)

    @staticmethod
    def edge(pulse, last, rising=True):
        status = (pulse if rising else not pulse) and pulse != last
        return status, pulse


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
    angle = round(math.atan2(y, x), 6)
    if angle < 0:
        angle += 2* math.pi
    return angle


def inTolerance(a, b, tol=1):
    return abs(a - b) <= tol


def limit(x, low, high):
    return min(high, max(x, low))


def getCoordinates(angle):
    # Convert the angle from degrees to radians
    angle = math.radians(angle)
    # Calculate the x and y coordinates
    x = round(math.cos(angle + math.pi/2), 3)
    y = round(math.sin(angle + math.pi/2), 3)
    # Return the coordinates as a tuple
    return x, y


def launchSmartDashboard(path="./Resources/shuffleboard.jar"):
    system(f"java -jar {path} >/dev/null 2>&1 &")

# ---- Bezier Curves :) ----


def quad_bezier(start, stop, ctrl):
    start, stop, ctrl = np.array([start, stop, ctrl])
    P = lambda t: (1-t)**2 * start + 2 * t * (1-t) * ctrl + t**2 * stop
    return np.array([P(t) for t in np.linspace(0, 1, 50)])


# find the a & b points
def get_bezier_coef(points):
    # since the formulas work given that we have n+1 points
    # then n must be this:
    n = len(points) - 1

    # build coefficents matrix
    C = 4 * np.identity(n)
    np.fill_diagonal(C[1:], 1)
    np.fill_diagonal(C[:, 1:], 1)
    C[0, 0] = 2
    C[n - 1, n - 1] = 7
    C[n - 1, n - 2] = 2

    # build points vector
    P = [2 * (2 * points[i] + points[i + 1]) for i in range(n)]
    P[0] = points[0] + 2 * points[1]
    P[n - 1] = 8 * points[n - 1] + points[n]

    # solve system, find a & b
    A = np.linalg.solve(C, P)
    B = [0] * n
    for i in range(n - 1):
        B[i] = 2 * points[i + 1] - A[i + 1]
    B[n - 1] = (A[n - 1] + points[n]) / 2

    return A, B


# returns the general Bezier cubic formula given 4 control points
def get_cubic(a, b, c, d):
    return lambda t: np.power(1 - t, 3) * a + 3 * np.power(1 - t, 2) * t * b + 3 * (1 - t) * np.power(t, 2) * c + np.power(t, 3) * d


# return one cubic curve for each consecutive points
def get_bezier_cubic(points):
    A, B = get_bezier_coef(points)
    return [
        get_cubic(points[i], A[i], B[i], points[i + 1])
        for i in range(len(points) - 1)
    ]


# evaluate each cubic curve on the range [0, 1] sliced in n points
def evaluate_bezier(points, n):
    return np.array([fun(t) for fun in get_bezier_cubic(points) for t in np.linspace(0, 1, n)])


def line2dots(line):
    x1, y1, x2, y2 = *line[0], *line[1]
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
    return np.linalg.norm(a-b)


# Detection stuff
def getLabelsFromTxt(path="coco-lbl.txt", verbose=True):
    with open(path, "r") as lbls:  # Open txt file
        a = lbls.read()  # Read txt file
        b = a.split('\n')  # Split by every line into list
        if verbose: print("Labels Extracted: ", b)  # print extracted list
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


def depth2xyzuv(depth, u=None, v=None):
    """
    Return a point cloud, an Nx3 array, made by projecting the kinect depth map
      through intrinsic / extrinsic calibration matrices
    Parameters:
      depth - comes directly from the kinect
      u,v - are image coordinates, same size as depth (default is the original image)
    Returns:
      xyz - 3D world coordinates in meters (Nx3)
      uv - image coordinates for the RGB image (Nx3)

    You can provide only a portion of the depth image, or a downsampled version of
      the depth image if you want; just make sure to provide the correct coordinates
      in the u,v arguments.

    Example:
      # This downsamples the depth image by 2 and then projects to metric point cloud
      u,v = mgrid[:480:2,:640:2]
      xyz,uv = depth2xyzuv(freenect.sync_get_depth()[::2,::2], u, v)

      # This projects only a small region of interest in the upper corner of the depth image
      u,v = mgrid[10:120,50:80]
      xyz,uv = depth2xyzuv(freenect.sync_get_depth()[v,u], u, v)
    """
    if u is None or v is None:
        u, v = np.mgrid[:480, :640]

    # Build a 3xN matrix of the d,u,v data
    C = np.vstack((u.flatten(), v.flatten(), depth.flatten(), 0 * u.flatten() + 1))

    # Project the duv matrix into xyz using xyz_matrix()
    X, Y, Z, W = np.dot(xyz_matrix(), C)
    X, Y, Z = X / W, Y / W, Z / W
    xyz = np.vstack((X, Y, Z)).transpose()
    xyz = xyz[Z < 0, :]

    # Project the duv matrix into U,V rgb coordinates using rgb_matrix() and xyz_matrix()
    U, V, _, W = np.dot(np.dot(uv_matrix(), xyz_matrix()), C)
    U, V = U / W, V / W
    uv = np.vstack((U, V)).transpose()
    uv = uv[Z < 0, :]

    # Return both the XYZ coordinates and the UV coordinates
    return xyz, uv


def uv_matrix():
    """
    Returns a matrix you can use to project XYZ coordinates (in meters) into
        U,V coordinates in the kinect RGB image
    """
    rot = np.array([[9.99846e-01, -1.26353e-03, 1.74872e-02],
                    [-1.4779096e-03, -9.999238e-01, 1.225138e-02],
                    [1.747042e-02, -1.227534e-02, -9.99772e-01]])
    trans = np.array([[1.9985e-02, -7.44237e-04, -1.0916736e-02]])
    m = np.hstack((rot, -trans.transpose()))
    m = np.vstack((m, np.array([[0, 0, 0, 1]])))
    KK = np.array([[529.2, 0, 329, 0],
                   [0, 525.6, 267.5, 0],
                   [0, 0, 0, 1],
                   [0, 0, 1, 0]])
    m = np.dot(KK, (m))
    return m


def xyz_matrix():
    fx = 594.21
    fy = 591.04
    a = -0.0030711
    b = 3.3309495
    cx = 339.5
    cy = 242.7
    mat = np.array([[1 / fx, 0, 0, -cx / fx],
                    [0, -1 / fy, 0, cy / fy],
                    [0, 0, 0, -1],
                    [0, 0, a, b]])
    return mat


if __name__ == "__main__":
    controller = XboxController()
    while True:
        print(controller.read())



