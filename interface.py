import math
import time
from threading import Thread
from utils import getAngle, smoothSpeed, inTolerance

try:
    import RPi.GPIO as io
    io.setmode(io.BCM)
    io.setwarnings(False)
except ImportError:
    sim = True



class Drive:  # TODO: Implement self.max properly
    def __init__(self, com='/dev/ttyACM0', baud=115200, mag=None, max_speed=1):
        self.lf = 0
        self.rf = 0
        self.lb = 0
        self.rb = 0
        self.max = max_speed
        self.mag = mag
        self.mecanum = 1
        self.distance_per_tick = 0.1427995  # in mm
        if not sim:
            import serial
            self.board = serial.Serial(com, baud) if not sim else NotImplemented
            self.comm_thread = Thread(target=self.comms)
            self.comm_thread.start()
        self.encoders = [0] * 4
        self.pos = [0, 0, 0]

    def cartesian(self, x, y, speed=1, turn=0):
        theta = math.radians(getAngle(x, y))
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

    def tank(self, x, power=1, turn=True):
        if power != 0:
            lf = lb = round((power + ((power / abs(power)) * min(0, x))), 3)
            rf = rb = round((power - ((power / abs(power)) * max(0, x))), 3)
        elif turn:
            lf = lb = 1*turn
            rf = rb = -lf
        else:
            lf, rf, lb, rb = 0, 0, 0, 0
        self.lf = lf
        self.rf = rf
        self.lb = lb
        self.rb = rb
        return lf, rf, lb, rb

    def moveTo(self, x, y, theta, speed=1, tolerance=0.1):
        while inTolerance(x, self.pos[0], tolerance) or inTolerance(y, self.pos[1], tolerance)\
                or inTolerance(theta, self.pos[2], tolerance):
            # Turn calculation is broken. Also remember to later implement actual map path planning.
            self.cartesian(smoothSpeed(self.pos[0], x),
                           smoothSpeed(self.pos[1], y),
                           speed, smoothSpeed(self.pos[2], theta))

    def comms(self):
        # This will most likely not work. Look for alternatives. Wiring everything to the pi isn't such a bad idea imo.
        while self.mecanum != 2:
            self.board.write(f"{self.lf},{self.rf},{self.lb},{self.rb}".encode("utf-8"))
            time.sleep(0.1)
            self.pos = self.board.read(4)

    def drive(self, x, y, power, turn):
        if self.mecanum:
            return self.cartesian(x, y, power, turn)
        else:
            return self.tank(x, power, turn)

    def switchDrive(self):
        self.mecanum = not self.mecanum

    def brake(self):
        self.lf, self.rf, self.lb, self.rb = 0, 0, 0, 0

    def exit(self):
        self.mecanum = 2
        if not sim:
            self.board.write(f"{0},{0},{0},{0}".encode("utf-8"))
            self.board.close()


class MPU:
    def __init__(self):
        import smbus
        from imusensor.MPU9250 import MPU9250
        self.mpu = MPU9250.MPU9250(bus=smbus.SMBus(1), address=0x68) if not sim else NotImplemented
        if not sim:
            self.mpu.begin()
            try:
                self.mpu.loadCalibDataFromFile("calib.json")
            except FileNotFoundError:
                pass
            self.values = [0] * 3
            readThread = Thread(target=self._update)
            readThread.start()

    def _update(self):
        while self.mpu != NotImplemented:
            self.mpu.readSensor()
            self.mpu.computeOrientation()

    def getMag(self):
        return self.mpu.MagVals if self.mpu != NotImplemented else 0

    def getGyro(self):
        return self.mpu.GyroVals if self.mpu != NotImplemented else 0

    def getAccel(self):
        return self.mpu.AccelVals if self.mpu != NotImplemented else 0

    def getAngle(self):
        return self.mpu.Yaw if self.mpu != NotImplemented else 0

    def exit(self):
        self.mpu = NotImplemented


class Lidar:
    def __init__(self):
        raise NotImplementedError("Lidar not implemented yet")


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
    def __init__(self, num_servos=6, smoothness=10):
        from adafruit_servokit import ServoKit
        self.smoothness = smoothness
        self.kit = ServoKit(channels=8 if num_servos >= 8 else 16)
        self.pose = [0] * num_servos
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
        while self.pose != pose:
            for i, joint in enumerate(self.arm):
                position = int(smoothSpeed(self.pose[i], pose[i], 10, 1, self.smoothness))
                self.pose[i] += position
                joint.angle = self.pose[i]
                time.sleep(0.005)


class Battery:
    # TODO: check if this works.
    def __init__(self):
        from adafruit_ina219 import INA219
        import board
        import busio
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.ina = INA219(self.i2c)

    def read(self):
        return self.ina.bus_voltage+self.ina.shunt_voltage, self.ina.current

# IO Left: Arduino Comms (In Drive), Networking (Special Case)
