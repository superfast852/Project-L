import time
from threading import Thread
from extensions.tools import getAngle, smoothSpeed, inTolerance, math
from breezyslam.sensors import RPLidarA1
from rplidar import RPLidar
from itertools import groupby
from operator import itemgetter
from Rosmaster_Lib import Rosmaster
driver = Rosmaster("/dev/ttyUSB0")
driver.create_receive_threading()


class Drive:  # TODO: Implement self.max properly
    def __init__(self, max_speed=1, collision_fn=lambda x: range(360), arg=None):
        self.lf = 0
        self.rf = 0
        self.lb = 0
        self.rb = 0
        self.max = max_speed
        self.mecanum = 1
        self.collision_fn = collision_fn
        self.arg = arg
        self.thread_life = 1
        self.comm_thread = Thread(target=self.comms)
        self.comm_thread.start()
        self.kinematics = MecanumKinematics((0, 0, 0, 0))
        self.test_speeds = ((1, 1, 1, 1), (-1, -1, -1, -1),  # Forward, Backward
                            (-1, 1, 1, -1), (1, -1, -1, 1),  # Left, Right
                            (0, 1, 1, 0), (-1, 0, 0, -1),    # TopLeft, BottomLeft
                            (1, 0, 0, 1), (0, -1, -1, 0),    # TopRight, BottomRight
                            (1, -1, 1, -1), (-1, 1, -1, 1))  # RotateRight, RotateLeft

    # Movement Functions
    def cartesian(self, x, y, speed=1, turn=0, flooring=False):
        if flooring:
            theta = math.radians(getAngle(x, y) % flooring)
        else:
            theta = math.radians(getAngle(x, y))
        if theta in self.collision_fn(self.arg):
            print("[WARNING] Drive: Would Collide!")
            self.brake()
            return 0, 0, 0, 0

        sin = math.sin(theta+math.pi/4)
        cos = math.cos(theta+math.pi/4)
        lim = max(abs(sin), abs(cos))

        lf = speed * cos / lim + turn
        rf = speed * sin / lim - turn
        lb = speed * sin / lim + turn
        rb = speed * cos / lim - turn

        if (speed + abs(turn)) > 1:
            lf /= speed + abs(turn)
            rf /= speed + abs(turn)
            lb /= speed + abs(turn)
            rb /= speed + abs(turn)

        lf = float(str(round(lf, 3))[0:5])
        rf = float(str(round(rf, 3))[0:5])
        lb = float(str(round(lb, 3))[0:5])
        rb = float(str(round(rb, 3))[0:5])

        self.lf = lf
        self.rf = rf
        self.lb = lb
        self.rb = rb

        return lf, rf, lb, rb

    def tank(self, x, y, power=1, turn=0):
        # TODO: Add turn directly onto function. Add Collision Detection.
        # collisions = self.collision_fn(self.arg)
        if power != 0:
            # The turn operation is sus. It will override power
            # l = y+(y/abs(y)*min(0,x))
            # r = y-(y/abs(y)*max(0,x))
            # Cases:  (y will always be 1)
            # 0:
            #   l = 1+(1/1*0) = 1
            #   r = 1-(1/1*0) = 1
            # -1:
            #   l = 1+(1/1*-1) = 0
            #   r = 1-(1/1*0) = 1
            # 1:
            #   l = 1+(1/1*0) = 1
            #   r = 1-(1/1*1) = 0
            if y < 0.2:
                power = -power
            lf = lb = round((power + (power / abs(power) * min(0, x))), 3)
            rf = rb = round((power - ((power / abs(power)) * max(0, x))), 3)
        elif turn != 0:
            lf = lb = round(turn, 3)
            rf = rb = round(-turn, 3)
        else:
            lf, rf, lb, rb = 0, 0, 0, 0
        self.lf = lf
        self.rf = rf
        self.lb = lb
        self.rb = rb
        return lf, rf, lb, rb

    def comms(self, update_freq=10):
        # This will most likely not work. Look for alternatives. Wiring everything to the pi isn't such a bad idea imo.
        while self.thread_life:
            driver.set_motor(self.lf, self.rf, self.lb, self.rb)
            time.sleep(1/update_freq)

    def drive(self, x, y, power, turn, flooring=False):
        if self.mecanum:
            return self.cartesian(x, y, power, turn, flooring)
        else:
            return self.tank(x, power, turn)

    def switchDrive(self):
        self.mecanum = not self.mecanum

    def brake(self):
        self.lf, self.rf, self.lb, self.rb = -self.lf, -self.rf, -self.lb, -self.rb
        time.sleep(0.1)
        self.lf, self.rf, self.lb, self.rb = 0, 0, 0, 0

    def smoothBrake(self, break_time=1):
        lfL = smoothSpeed(self.lf, 0)
        rfL = smoothSpeed(self.rf, 0)
        lbL = smoothSpeed(self.lb, 0)
        rbL = smoothSpeed(self.rb, 0)
        for i in range(101):
            self.lf, self.rf, self.lb, self.rb = lfL(i), rfL(i), lbL(i), rbL(i)
            time.sleep(break_time/100)

    # Advanced Movement Functions
    # Add NewSmooth
    def smoothMove(self, x, y, theta, speed=1, tolerance=0.1):
        coords = (x, y, theta)
        smoothers = [smoothSpeed(self.kinematics.pose[i], coords[i]) for i in [0, 1, 2]]
        diffs = [self.kinematics.pose[i] - coords[i] for i in [0, 1, 2]]

        while (inTolerance(x, self.kinematics.pose[0], tolerance) or inTolerance(y, self.kinematics.pose[1], tolerance)
                or inTolerance(theta, self.kinematics.pose[2], tolerance)):
            # Turn calculation is broken. Also remember to later implement actual map path planning.

            progs = [(1-(self.kinematics.pose[i] - coords[i])/diffs[i])*100 for i in [0, 1, 2]]
            self.cartesian(smoothers[0](progs[0]), smoothers[1](progs[1]), speed, smoothers[2](progs[2]))

    def moveTo(self, x, y, theta, speed=1, tolerance=0.1, wait=0):
        xDir = 0
        yDir = 0
        turnDir = 0
        waiting = 1
        while waiting:
            if not inTolerance(x, self.kinematics.pose[0], tolerance):
                if x-self.kinematics.pose[0] > 0:
                    xDir = -1
                else:
                    xDir = 1
            if not inTolerance(y, self.kinematics.pose[1], tolerance):
                if y-self.kinematics.pose[1] > 0:
                    yDir = -1
                else:
                    yDir = 1
            if not inTolerance(theta, self.kinematics.pose[2], tolerance):
                if theta-self.kinematics.pose[2] > 0:
                    turnDir = -1
                else:
                    turnDir = 1
            out = self.cartesian(xDir, yDir, speed, turnDir)
            if (xDir == 0 and yDir == 0 and turnDir == 0) or out == (0, 0, 0, 0):
                break
            waiting = wait

    def exit(self):
        self.brake()
        time.sleep(0.1)
        self.thread_life = 0


class MPU:
    def __init__(self):
        self.mag = 0
        self.gyro = 0
        self.acc = 0
        self.thread_life = 1

    def _update(self):
        while self.thread_life:
            self.mag = driver.get_magnetometer_data()
            self.gyro = driver.get_gyroscope_data()
            self.acc = driver.get_accelerometer_data()
            time.sleep(0.1)

    def getMag(self):
        return self.mag

    def getGyro(self):
        return self.gyro

    def getAccel(self):
        return self.acc

    def exit(self):
        self.thread_life = 0


class Ultrasonic:
    # TODO: Check if it works.
    def __init__(self, echo, trig):  # 11, 8
        self.echo = echo
        self.trig = trig
        io.setup(self.echo, io.IN)
        io.setup(self.trig, io.OUT)

    def _get_val(self):
        # set Trigger to HIGH
        io.output(self.trig, True)

        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        io.output(self.trig, False)

        StartTime = time.time()
        StopTime = time.time()

        # save StartTime
        while io.input(self.echo) == 0:
            StartTime = time.time()

        # save time of arrival
        while io.input(self.echo) == 1:
            StopTime = time.time()

        distance = ((StopTime-StartTime) * 34300) / 2

        return distance

    def read(self, measures=5):
        return sum([self._get_val() for i in range(measures)])/measures


class Arm:
    def __init__(self, num_servos=6, lapse=1, steps=100):
        from adafruit_servokit import ServoKit
        self.delay = lapse/steps
        self.steps = steps
        self.pose = [0] * num_servos
        self.kit = ServoKit(channels=8 if num_servos >= 8 else 16)
        self.arm = [self.kit.servo[i] for i in range(num_servos)]
        self.home = [90, 75, 130, 90, 150, 180]
        self.grabbing = [90, 10, 90, 100, 150, 180]
        self.dropping = [90, 50, 20, 0, 150, 0]
        self.move(self.home)

    def grab(self):
        self.pose[-1] = 0
        self.arm[-1].angle = 0

    def drop(self):
        self.pose[-1] = 180
        self.arm[-1].angle = 180

    def grab_item(self, period=0.5):
        self.move(self.grabbing)
        time.sleep(period)
        self.grab()
        time.sleep(0.1)
        self.move(self.dropping)
        time.sleep(period)
        self.drop()
        time.sleep(0.1)
        self.move(self.home)

    def move(self, pose=None):
        if pose is None:
            pose = self.pose

        move_prof = [smoothSpeed(self.pose[i], pose[i], self.steps) for i in range(len(self.pose))]
        for i in range(self.steps):
            for n in range(len(self.arm)):
                self.arm[n].angle = int(move_prof[n][i])
            time.sleep(self.delay)


class Battery:
    # TODO: check if this works.
    def __init__(self):
        self.batt = 0

    def read(self):
        self.batt = driver.get_battery_voltage()
        return self.batt


class RP_A1(RPLidarA1):
    def __init__(self, com="/dev/ttyUSB0", baudrate=115200, timeout=3, rotation=0):
        super().__init__()
        self.lidar = RPLidar(com, baudrate, timeout)
        print(self.lidar.get_info(), self.lidar.get_health())
        self.scanner = self.lidar.iter_scans()
        next(self.scanner)
        self.map = [0]*360
        self.rotation = rotation

    def read(self):
        items = [item for item in next(self.scanner)]
        angles = [item[1] for item in items]
        distances = [item[2] for item in items]
        if self.rotation != 0:
            distances, angles = list(zip(*self.rotate_lidar_readings(zip(distances, angles), self.rotation)))
        for i in range(len(angles)):
            try:
                self.map[int(angles[i])] = distances[i]
            except IndexError:
                pass
        return angles, distances

    def exit(self):
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()

    def autoStopCollision(self, collision_threshold):
        return [i for i, distance in enumerate(self.map) if 0 < distance < collision_threshold]

    def self_nav(self, collision_angles, collision_threshold, lMask=(180, 271), rMask=(270, 361)):
        if min(self.map[collision_angles[0]:collision_angles[1]]) < collision_threshold:
            ls_clear = []
            rs_clear = []

            for i, distance in enumerate(self.map[rMask[0]:rMask[1]]):  # Right Side Check
                if distance > collision_threshold:  # If the vector_distance is greater than the threshold
                    rs_clear.append(i + rMask[0])  # append the angle of the vector to the list of available angles

            for i, distance in enumerate(self.map[lMask[0]:lMask[1]]):  # Left Side Check
                if distance > collision_threshold:
                    ls_clear.append(i + lMask[0])

            # From the list, extract all numerical sequences (e.g. [3, 5, 1, 2, 3, 4, 5, 8, 3, 1] -> [[1, 2, 3, 4, 5]])
            ls_sequences = self._extract_sequence(ls_clear)
            rs_sequences = self._extract_sequence(rs_clear)

            # Find the longest sequence, as there might be multiple sequences.
            # If there were no sequences, the longest sequence would be 0.
            ls_space = max(ls_sequences, key=len) if len(ls_sequences) > 0 else [0, 0]
            rs_space = max(rs_sequences, key=len) if len(rs_sequences) > 0 else [0, 0]

            if len(ls_space) > len(rs_space):  # If the left side has more space
                return "Left", ls_space  # Return the direction and the angles of the obstacle for future optional odometry.
            else:  # If the right side has more space
                return "Right", rs_space  # Return the direction and the angles of the obstacle for future optional odometry.
        else:
            return "Forward", self.map[collision_angles[0]:collision_angles[1]]

    @staticmethod
    def _extract_sequence(array):
        sequences = []
        for k, g in groupby(enumerate(array), lambda x: x[0] - x[1]):
            seq = list(map(itemgetter(1), g))
            sequences.append(seq) if len(seq) > 1 else None
        return sequences

    def rotate_lidar_readings(self, readings):
        """
        Rotate lidar angle readings by the specified angle while keeping distances intact.

        Args:
            readings (list): List of tuples containing distances and angles.

        Returns:
            list: Rotated lidar readings with preserved distances.
        """
        if self.rotation % 360 == 0:
            return readings
        else:
            return [(readings[i][0], (readings[i][1] + self.rotation + 360) % 360) for i in range(len(readings))]


class MecanumKinematics:  # TODO: ADD THIS PROPERLY
    def __init__(self, ticks, radius=1, wheel2centerDistance=1):
        self.r = radius
        self.ticks = list(ticks)
        self.w2c = wheel2centerDistance
        self.x = lambda lf, rf, lb, rb: (lf - rf - lb + rb) * (self.r / 4)
        self.y = lambda lf, rf, lb, rb: (lf + rf + lb + rb) * (self.r / 4)
        self.w = lambda lf, rf, lb, rb: (lf - rf + lb - rb) * (
                    self.r / (4 * self.w2c))
        self.pose = [0, 0, 0]
        self.thread = Thread(target=self.updatePosition)
        self.thread.start()

    def updatePosition(self):
        while True:
            self.ticks = driver.get_motor_encoder()
            self.pose[0] = self.x(self.ticks[0], self.ticks[1], self.ticks[2], self.ticks[3])
            self.pose[1] = self.y(self.ticks[0], self.ticks[1], self.ticks[2], self.ticks[3])
            self.pose[2] = self.w(self.ticks[0], self.ticks[1], self.ticks[2], self.ticks[3])
            time.sleep(1/30)

    def getTicks(self, x, y, w):
        recip = 1 / self.r
        turn = self.w2c * w
        xy = x + y
        notxy = x - y
        return [recip * (notxy - turn), recip * (xy + turn),
                recip * (xy - turn), recip * (notxy + turn)]
