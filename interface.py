import math
import time
from threading import Thread
from utils import getAngle, smoothSpeed, inTolerance

try:
    import RPi.GPIO as io
except ImportError:
    from RPiSim import GPIO as io
    io.IN = io.MODE_IN
    io.OUT = io.MODE_OUT
    sim = True

io.setmode(io.BCM)
io.setwarnings(False)


class Drive:
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

    def cartesian(self, x, y, speed, turn):
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

    def tank(self, x, power, turn=True):
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

    def moveTo(self, x, y, theta, speed, tolerance=0.1):
        while inTolerance(x, self.pos[0], tolerance) or inTolerance(y, self.pos[1], tolerance)\
                or inTolerance(theta, self.pos[2], tolerance):

            self.cartesian(smoothSpeed(self.pos[0], x),
                           smoothSpeed(self.pos[1], y),
                           speed, smoothSpeed(self.pos[2], theta))

    def comms(self):
        while self.mecanum != 2:
            self.board.write(f"{self.lf},{self.rf},{self.lb},{self.rb}".encode("utf-8"))
            time.sleep(0.1)
            self.encoders = self.board.read(4)
            self.calc_odometry()

    def calc_odometry(self):
        self.pos[0] += (self.encoders[0] + self.encoders[1] + self.encoders[2] + self.encoders[3]) * self.distance_per_tick / 4
        self.pos[1] += (self.encoders[0] + self.encoders[1] - self.encoders[2] - self.encoders[3]) * self.distance_per_tick / 4
        self.pos[2] = self.mag.getAngle() if self.mag else 0

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
    def __init__(self):
        raise NotImplementedError("Arm not implemented yet")

    def move(self, pose):
        pass


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


class GPS:
    def __init__(self):
        raise NotImplementedError("GPS not implemented yet")

    def read(self):
        pass

# IO Left: Arduino Comms (In Drive), Networking (Special Case)
