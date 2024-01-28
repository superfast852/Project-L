from extensions.NavStack import Map, RRT, SLAM
from extensions.FastCam import Camera
from extensions.Detector import Detector, np
import cv2
from _pickle import load
from time import perf_counter


class BenchBot:
    # Slam Algorithm Time: 0.15378022193908691
    def __init__(self):
        self.detector = Detector("./Resources/yolov7-tiny-nms.trt", track=False)
        self.cam = Camera()
        self.cam.start()
        self.map = Map("random")
        self.rrt = RRT(Map("random"))
        self.slam = SLAM(None, self.map)

        with open("./Resources/test_scans.pkl", "rb") as f:
            self.scans = load(f)
        self.scan_counter = 0
        self.frame = np.ones((480, 640, 3))*255
        self.pose = (0, 0, 0)

    def test(self):  # 214.31513808763586

        print("[1/4] Benchmarking Object Detection W/O Tracking...")
        det_times = self.measured_loops(self.detect)
        self.detector.track = True
        print("[2/4] Benchmarking Object Detection W/ Tracking...")
        det_times_tracked = self.measured_loops(self.detect)
        print(f"[3/4] Benchmarking SLAM Algorithm...")
        slam_times = self.measured_loops(self.slamops)
        draw_times = [1/100]
        totals = []
        print(f"[4/4] Benchmarking Consecutive Payload...")
        self.map = Map("random")
        for i in range(1000):
            print(i)
            start = perf_counter()
            self.detect()
            self.slamops()
            draw_times.append(self.draw())
            totals.append(perf_counter()-start)
        print(f"Totals:\n\tDetections: {len(det_times)/sum(det_times):.3f}\n\tSLAM: {len(slam_times)/sum(slam_times):.3f}"
              f"\n\tDraw: {len(draw_times)/sum(det_times):.3f}\n\tDetections (Tracked): {len(det_times_tracked)/sum(det_times_tracked):.3f}\n\t"
              f"Consecutive Framerate: {len(totals)/sum(totals):.3f}")

    def measured_loops(self, f, n=1000):
        totals = []
        for i in range(n):
            start = perf_counter()
            f()
            totals.append(perf_counter()-start)
        return totals

    def detect(self):
        frame = self.cam.read()
        dets = self.detector(frame)
        if dets is not None:
            dets.draw(frame)
        self.frame = frame

    def rrtops(self):  # please note, try these independently, as it's supposed to be used sparsely.
        tstart = perf_counter()
        start = self.map.getValidPoint()
        stop = self.map.getValidPoint()
        path = self.rrt.plan(start, stop)
        return perf_counter()-tstart

    def slamops(self):
        self.pose = self.slam.update(self.scans[self.scan_counter % (len(self.scans) - 1)])

    def draw(self):
        start = perf_counter()
        cv2.imshow("ai", self.frame)
        self.map.animate(None, self.pose)
        return perf_counter()-start


if __name__ == "__main__":
    # Pre optimization value: 152.1791710853688 ms
    # POST OPTIMIZATION VALUE: 3.448326349258423 ms
    # og new method: 0.0035310662984848022
    # alt method 1: 0.003485587811470032 (10k iters)
    # HOLY SHIT
    bench = BenchBot()
    timing = []
    for i in range(100):
        print(f"Iteration: {i}")
        timing.append(bench.rrtops())
    bench.cam.stop()
    print(f"RRT Performance: {len(timing)/sum(timing)}")
