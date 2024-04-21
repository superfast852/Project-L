from threading import Thread
import cv2
import freenect
from extensions.tools import depth2xyzuv, np


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


class Kinect:
    def __init__(self):
        import freenect
        print("Freenect is imported!")

    def getDepthFrame(self):
        # X: Side-To Side, Y: Up-Down, Z: Depth
        return freenect.sync_get_depth()[0].astype(np.uint8)

    def getVideo(self):
        return cv2.cvtColor(freenect.sync_get_video()[0].astype(np.uint8), cv2.COLOR_RGB2BGR)

    def setVideoMode(self):
        freenect.set_video_mode(freenect.RESOLUTION_MEDIUM, freenect.VIDEO_RGB)