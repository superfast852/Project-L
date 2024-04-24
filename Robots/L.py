
# All of this is very much broken. When implementing, remember to fix this mess.

from RM_HAL import Drive, RP_A1, Arm, Battery, IMU, time
from extensions.NavStack import Map, RRT, SLAM
from extensions.PolyTask import TaskManager
from extensions.Detector import Detector, coco
from extensions.FastCam import Camera
from networktables import NetworkTables


"""
class Robot:
    def __init__(self, manager=None, show_map=True):
        NetworkTables.initialize()
        if manager is None:
            manager = TaskManager()

        self.manager = manager
        # Components:
        self.drive = Drive()  # uses rosmaster
        self.mpu = MPU()
        self.battery = Battery(self.manager)
        self.lidar = RP_A1()  # uses lidar
        self.arm = Arm()  # uses servokit
        self.cam = Camera().start()

        # Algos:
        self.map = Map(800)
        self.rrt = RRT(self.map)
        self.slam = SLAM(self.lidar, self.map)
        self.detector = Detector("../Resources/yolov7-tiny-nms.trt")
        self.bgID = manager.add_task(self.background, (show_map,), type="async")
        self.detID = manager.add_task(self.register_detections, type="async")
        # self.bgThread = Thread(target=self.background, args=(show_map,), daemon=True)
        # self.bgThread.start()
        # self.detThread = Thread(target=self.background, args=(show_map,), daemon=True)
        # self.detThread.start()

        # Data:
        data = NetworkTables.getTable("data")
        images = NetworkTables.getTable("image")
        data.putNumberArray("pose", (0, 0, 0))
        data.putNumberArray("scan", [0, 0, 0, 0, 0])
        images.putNumberArray("frame", [[[0, 0, 0], [0, 0, 0]], [[0, 0, 0], [0, 0, 0]], [[0, 0, 0], [0, 0, 0]]])

    def background(self, show_map=True):
        start = time.time()
        data = NetworkTables.getTable("data")
        image = NetworkTables.getTable("image")
        while True:
            angles, distances = self.lidar.read()
            data.putNumberArray("scan", self.lidar.map.copy())
            pose = self.slam.update(distances, angles, self.drive.kinematics.computePoseChange(start-time.time()))
            data.putNumberArray("pose", pose)
            if show_map:
                self.map.animate(pose=pose)
            image.putNumberArray("frame", self.cam.read())

    def goTo(self, x, y, table):  # TODO: WARNING. THIS WILL NOT WORK.
        # The issue lies in using both lidar pose and kinematic pose.
        # The lidar pose is used to determine our point on the map, and kinematic pose is used in drive.moveTo
        pose = table.getNumberArray("pose")
        path = self.rrt.plan(pose[:2], (x, y))
        self.map.addPath(path)
        ids = []
        for line in path:
            goal = line[1]
            ids.append(manager.add_task(self.drive.moveTo, args=(goal[0], goal[1], pose[2])))
        return ids  # This makes it so that you have to wait for the task to finish before controlling the motors again.

    def moveArmCoords(self, x, y, z):  # inverse kinematics. Yeah, not now.
        pass

    def moveArmAngles(self, joints, mode="async"):
        return manager.add_task(self.arm.move, (joints,), type=mode)

    def register_detections(self):
        images = NetworkTables.getTable("image")
        while True:
            dets = self.detector.get_dets(images.getNumberArray("frame"))
            images.putNumberArray("dets", dets)
            for det in dets:
                box, tag, score, classing = det
                if images.getNumberArray(coco[classing], None) is None:
                    images.putNumberArray(coco[classing], images.getNumberArray("pose"))
"""


class Robot:
    def __init__(self, manager=None, show_map=True):
        NetworkTables.initialize()
        if manager is None:
            manager = TaskManager()

        self.manager = manager
        # Components:
        self.drive = Drive()  # uses rosmaster
        self.imu = IMU()
        self.battery = Battery(self.manager)
        self.lidar = RP_A1()  # uses lidar
        self.arm = Arm()  # uses servokit
        self.cam = Camera().start()

        # Algos:
        self.map = Map(800)
        self.rrt = RRT(self.map)
        self.slam = SLAM(self.lidar, self.map)
        self.detector = Detector("../Resources/yolov7-tiny-nms.trt")
        self.bgID = manager.add_task(self.background, (show_map,), type="async")
        self.detID = manager.add_task(self.register_detections, type="async")
        # self.thread = Thread(target=self.background, args=(show_map,), daemon=True)
        # self.thread.start()

        # Data:
        self.pose = (0, 0, 0)
        self.scan = []
        self.frame = []
        self.items = {}
        self.dets = []

    def background(self, show_map=True):
        start = time.time()
        while True:
            angles, distances = self.lidar.read()
            self.scan = self.lidar.map.copy()
            pose = self.slam.update(distances, angles, self.drive.kinematics.computePoseChange(start-time.time()))
            self.pose = pose
            if show_map:
                self.map.animate(pose=pose)
            self.frame = self.cam.read()

    def goTo(self, x, y):  # TODO: WARNING. THIS WILL NOT WORK.
        # The issue lies in using both lidar pose and kinematic pose.
        # The lidar pose is used to determine our point on the map, and kinematic pose is used in drive.moveTo
        path = self.rrt.plan(self.pose[:2], (x, y))
        if not self.rrt.isValidPath(path):
            return []
        self.map.addPath(path)
        ids = []
        for line in path:
            goal = line[1]
            ids.append(manager.add_task(self.drive.moveTo, args=(goal[0], goal[1], self.pose[2])))
        return ids  # This makes it so that you have to wait for the task to finish before controlling the motors again.

    def moveArmCoords(self, x, y, z):  # inverse kinematics. Yeah, not now.
        pass

    def moveArmAngles(self, joints, mode="async"):
        return manager.add_task(self.arm.move(joints), type=mode)

    def register_detections(self):
        while True:
            dets = self.detector.get_dets(self.frame)
            for det in dets:
                box, tag, score, classing = det
                self.dets = dets
                if coco[classing] not in self.items.keys():
                    self.items[coco[classing]] = self.pose


if __name__ == '__main__':
    manager = TaskManager()
    bot = Robot(manager)

    while True:
        pass
