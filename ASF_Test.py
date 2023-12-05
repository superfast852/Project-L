import time

from extensions.NavStack import Map, RRT, SLAM
from extensions.FastCam import Camera
from extensions.Detector import Detector, np
import cv2
from _pickle import load
from time import perf_counter

class BenchmarkingTest:
    def __init__(self, test_names, test_funcs, verbose_mask, iterations):
        self.test_names = test_names
        self.test_funcs = test_funcs
        if isinstance(iterations, int):
            self.iterations = [iterations for _ in range(len(test_funcs))]
        else:
            self.iterations = iterations

        if isinstance(verbose_mask, bool):
            self.verbose_mask = [verbose_mask for _ in range(len(test_funcs))]
        else:
            self.verbose_mask = verbose_mask

    def benchmark(self):
        times = []
        for i, test in enumerate(self.test_funcs):
            print(f"[{i+1}/{len(self.test_funcs)}] Benchmarking {self.test_names[i]} for {self.iterations[i]} iterations...")
            times.append(self.iterative_tests(test, self.verbose_mask[i], self.iterations[i]))
        print("Benchmarking Complete!")
        print("Results:")
        for i, test in enumerate(self.test_names):
            print(f"\t{test}: {times[i]:.3f} FPS")

    def iterative_tests(self, f, v, n=100):
        totals = []
        for i in range(n):
            start = perf_counter()
            f()
            totals.append(perf_counter() - start)
            if v:
                print(i)
        return n/sum(totals)


class SoftwareTests(BenchmarkingTest):
    # Slam Algorithm Time: 0.15378022193908691
    def __init__(self):
        super().__init__(["Object Detection & Tracking", "SLAM Algorithm", "SLAM and YOLO"],
                         [self.detect, self.slamops, self.draw],
                         [False, False, False],
                         [1000, 1000, 1000])
        self.detector = Detector("./Resources/yolov7-tiny-nms.trt")
        self.cam = Camera()
        self.cam.start()
        self.map = Map(800)
        self.rrt = RRT(Map("random"))
        self.slam = SLAM(None, self.map)

        with open("./Resources/test_scans.pkl", "rb") as f:
            self.scans = load(f)
        self.scan_counter = 0
        self.frame = np.ones((480, 640, 3))*255
        self.pose = (0, 0, 0)
        self.paths = []

    def test(self):  # 214.31513808763586
        print("[1/4] Benchmarking Object Detection & Tracking...")
        det_times = self.measured_loops(self.detect)
        print("[2/4] Benchmarking RRT (100 iterations)...")
        det_times_tracked = self.measured_loops(self.rrtops, 100)
        print(f"[3/4] Benchmarking SLAM Algorithm...")
        slam_times = self.measured_loops(self.slamops)
        draw_times = [1/100]
        totals = []

        print(f"[4/4] Benchmarking SLAM and YOLO...")
        for i in range(1000):
            print(i)
            start = perf_counter()
            self.detect()
            self.slamops()
            draw_times.append(self.draw())
            totals.append(perf_counter()-start)
        print(f"Totals:\n\tDetections: {len(det_times)/sum(det_times):.3f}\n\tSLAM: {len(slam_times)/sum(slam_times):.3f}"
              f"\n\tDraw: {len(draw_times)/sum(det_times):.3f}\n\tRRT: {len(det_times_tracked)/sum(det_times_tracked):.3f}\n\t"
              f"Consecutive Framerate: {len(totals)/sum(totals):.3f}")

    def measured_loops(self, f, n=1000):
        totals = []
        for i in range(n):
            start = perf_counter()
            f()
            totals.append(perf_counter()-start)
            print(i)
        return totals

    def detect(self):
        frame = self.cam.read()
        dets = self.detector(frame)
        if dets is not None:
            dets.draw(frame)
        self.frame = frame

    def rrtops(self):  # please note, try these independently, as it's supposed to be used sparsely.
        start = self.map.getValidPoint()
        stop = self.map.getValidPoint()
        return self.rrt.plan(start, stop)

    def slamops(self):
        self.pose = self.slam.update(self.scans[self.scan_counter % (len(self.scans) - 1)])

    def draw(self):
        start = perf_counter()
        cv2.imshow("ai", self.frame)
        self.map.animate(None, self.pose)
        return perf_counter()-start

    def r_detect(self):
        self.detect()
        return self.frame

    def r_slam(self):
        self.slamops()
        return self.pose


class HardwareTests(BenchmarkingTest):
    def __init__(self):
        from Robots.RM_HAL import Drive, MPU, Battery, MecanumKinematics, driver, np, time
        super().__init__(["Drive", "MPU", "Battery"],
                         [self.driveTest, self.mpuTest, self.batTest],
                         [False, False, False],
                         [360, 1000, 1000])
        self.drive = Drive()
        self.driver = driver
        self.mpu = MPU()
        self.bat = Battery()
        self.vLevel = 12.6
        self.mag = []
        self.gyro = []
        self.accel = []
        self.trajectories = [(np.cos(i*np.pi/180), np.sin(i*np.pi/180))for i in range(360)]
        self.traj_count = 0


    def batTest(self):
        self.vLevel = sum([self.bat.get_voltage() for _ in range(10)]) / 10

    def mpuTest(self):
        self.mag.append(self.mpu.getMag())
        self.gyro.append(self.mpu.getGyro())
        self.accel.append(self.mpu.getAccel())

    def driveTest(self):
        traj = self.trajectories[self.traj_count % 360]
        print(traj)
        self.drive.cartesian(traj[0], traj[1], 1, 0)
        self.traj_count += 1
        time.sleep(0.01)



if __name__ == "__main__":
    # Pre optimization value: 152.1791710853688 ms
    # POST OPTIMIZATION VALUE: 3.448326349258423 ms
    # og new method: 0.0035310662984848022
    # alt method 1: 0.003485587811470032 (10k iters)
    # HOLY SHIT
    softbench = SoftwareTests()
    hardbench = HardwareTests()
    softbench.benchmark()
    hardbench.benchmark()
    print("Braking smoothly...")
    hardbench.drive.smoothBrake(10)
    hardbench.drive.exit()
