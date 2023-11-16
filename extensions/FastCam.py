from threading import Thread
import cv2
import datetime


class Camera:
    def __init__(self, src=0, res=None):
        self.stream = cv2.VideoCapture(src)
        if res is not None:
            self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, res[0])
            self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, res[1])
        (self.grabbed, self.frame) = self.stream.read()
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
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        try:
            return self.frame.copy()
        except AttributeError:
            print("Frame not found.")

    def stop(self):
        self.stopped = True

    def get(self):
        return self.stream.get(cv2.CAP_PROP_FRAME_WIDTH), self.stream.get(cv2.CAP_PROP_FRAME_HEIGHT)


class FPS:
    def __init__(self):
        self.start_time = None
        self.end = None
        self.numFrames = 0

    def start(self):
        self.start_time = datetime.datetime.now()
        return self

    def stop(self):
        self.end = datetime.datetime.now()

    def update(self):
        self.numFrames += 1

    def elapsed(self):
        return (self.end - self.start_time).total_seconds()

    def fps(self):
        return self.numFrames / self.elapsed()


if __name__ == '__main__':
    stream = Camera().start()
    fps = FPS().start()

    while fps.numFrames < 1000:
        frame = stream.read()
        cv2.imshow('Frame', frame)
        fps.update()

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    fps.stop()
    print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

    stream.stop()
    cv2.destroyAllWindows()