from threading import Thread
import cv2
from extensions.tools import np


class Camera:
    def __init__(self, src=0, res=None):
        self.stream = cv2.VideoCapture(src)
        if res is not None:
            self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, res[0])
            self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, res[1])
        self.grabbed, self.frame = self.stream.read()
        self.stopped = False
        self.main_thread = Thread(target=self.update, daemon=True)

    def start(self):
        self.main_thread.start()
        return self

    def __del__(self):
        self.stop()
        self.main_thread.join()

    def update(self):
        while True:
            if self.stopped:
                return
            self.grabbed, self.frame = self.stream.read()

    def read(self):
        try:
            return self.frame.copy()
        except AttributeError:
            print("Frame not found.")

    def stop(self):
        self.stopped = True

    def get(self):
        return self.stream.get(cv2.CAP_PROP_FRAME_WIDTH), self.stream.get(cv2.CAP_PROP_FRAME_HEIGHT)


class Stereo:
    def __init__(self, srcs=(0, 1), depth_params=None, **kwargs):
        self.l = cv2.VideoCapture(self.gstreamer_pipeline(srcs[0], **kwargs), cv2.CAP_GSTREAMER)
        self.r = cv2.VideoCapture(self.gstreamer_pipeline(srcs[1], **kwargs), cv2.CAP_GSTREAMER)
        self.depth_params = depth_params
        self.stereo = self.buildStereoBM(**depth_params)
        a = self.l.read()
        b = self.r.read()
        self.grabbed = a[0] and b[0]
        self.frames = [a[1], b[1], self.getDepth()]

        self.stopped = -1
        self.main_thread = Thread(target=self.update, daemon=True)
        self.depth_frame = Thread(target=self.depth_update, daemon=True)

    def start(self):
        if self.stopped == -1:
            self.main_thread.start()
            self.depth_frame.start()
            self.stopped = 0
        else:
            print("Stereo already started.")
        return self

    def __del__(self):
        self.stop()
        self.main_thread.join()
        self.depth_frame.join()

    def update(self):
        while not self.stopped:
            a = self.l.read()
            b = self.r.read()
            self.grabbed = a[0] and b[0]
            self.frames[0] = a[1]
            self.frames[1] = b[1]

    def depth_update(self):
        while not self.stopped:
            self.frames[2] = self.getDepth()


    def getDepth(self):
        return self.stereo.compute(cv2.cvtColor(self.frames[0], cv2.COLOR_BGR2GRAY),
                                   cv2.cvtColor(self.frames[1], cv2.COLOR_BGR2GRAY))

    def read(self, id=0):
        try:
            return self.frames[id].copy()
        except IndexError:
            print("[FastCam][ERROR]: Invalid frame ID.")

    def stop(self):
        self.stopped = 1

    @staticmethod
    def gstreamer_pipeline(sensor_id=0, capture_width=1920, capture_height=1080,
                           display_width=1920, display_height=1080, framerate=30, flip_method=0):
        return (
                f"nvarguscamerasrc sensor-id={sensor_id} ! "
                f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, framerate=(fraction){framerate}/1 ! "
                f"nvvidconv flip-method={flip_method} ! "
                f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
                "videoconvert ! "
                "video/x-raw, format=(string)BGR ! appsink"
        )

    @staticmethod
    def buildStereoBM(num_disparities=16, block_size=15):
        '''
        Additional Parameters:

            texture_threshold:
            filters out areas that don't have enough texture for reliable matching

            \nSpeckle range and size:
            Block-based matchers often produce "speckles" near the boundaries of objects,
            where the matching window catches the foreground on one side and the background on the other.
            In this scene it appears that the matcher is also finding small spurious matches in the projected
            texture on the table. To get rid of these artifacts we post-process the disparity image with a speckle
            filter controlled by the speckle_size and speckle_range parameters. speckle_size is the number of pixels
            below which a disparity blob is dismissed as "speckle." speckle_range controls how close in value disparities
            must be to be considered part of the same blob.

            \nNumber of disparities:
            Larger number gets more detail, which requires more computation. It must be divisible by 16.

            \nmin_disparity:
            the offset from the x-position of the left pixel at which to begin searching.

            \nuniqueness_ratio:
            Another post-filtering step. If the best matching disparity is not sufficiently better
            than every other disparity in the search range, the pixel is filtered out. You can try tweaking this if
            texture_threshold and the speckle filtering are still letting through spurious matches.

            \nprefilter_size and prefilter_cap:
            The pre-filtering phase, which normalizes image brightness and enhances
            texture in preparation for block matching. Normally you should not need to adjust these.

        :param num_disparities:
        :param block_size:
        :return: StereoBM object
        '''
        return cv2.StereoBM.create(num_disparities, block_size)