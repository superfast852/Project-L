"""HAL: Hardware Abstraction Layer"""

import math
import time
from threading import Thread
from tools import getAngle, smoothSpeed, inTolerance
from breezyslam.sensors import Laser, RPLidarA1
from numpy.random import randn
from serial import Serial
from typing import Tuple
from rplidar import RPLidar

try:
    import RPi.GPIO as io
    io.setmode(io.BCM)
    io.setwarnings(False)
    sim = False
except ImportError:
    sim = True
    class io:
        def cleanup(self):
            pass
    io = io()


class Drive:  # TODO: Implement self.max properly
    def __init__(self, com='/dev/ttyACM0', baud=115200, mag=None, max_speed=1):
        # IDEA: Use the MPU to get the velocities. Then, use Inverse Kinematics to get individual wheel speeds.
        self.lf = 0
        self.rf = 0
        self.lb = 0
        self.rb = 0
        self.max = max_speed
        self.mag = mag
        self.mecanum = 1
        self.geometry = {"radius": 0.5, "x-center distance": 0.5, "y-center distance": 0.5, "tpr": 1100,
                         "dpt": 0.1427995, "w": 200, "h": 300}
        if not sim:
            import serial
            self.board = serial.Serial(com, baud)
            self.comm_thread = Thread(target=self.comms)
            self.comm_thread.start()
        self.positioner = Positioning()

    def cartesian(self, x, y, speed=1, turn=0, flooring=False):
        if flooring:
            theta = math.radians(int(getAngle(x, y)/10)*10)
        else:
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

    def tank(self, x, power=1, turn=0):
        # TODO: Add turn directly onto function
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
            lf = lb = round((power + (power / abs(power) * min(0, x))), 3)
            rf = rb = round((power - ((power / abs(power)) * max(0, x))), 3)
        else:
            lf, rf, lb, rb = 0, 0, 0, 0
        self.lf = lf
        self.rf = rf
        self.lb = lb
        self.rb = rb
        return lf, rf, lb, rb

    def smoothMove(self, x, y, theta, speed=1, tolerance=0.1, wait=0):
        # TODO: Implement Async?
        waiting = 1
        while (inTolerance(x, self.positioner.pose[0], tolerance) or inTolerance(y, self.positioner.pose[1], tolerance)\
                or inTolerance(theta, self.positioner.pose[2], tolerance)) and waiting:
            # Turn calculation is broken. Also remember to later implement actual map path planning.
            self.cartesian(smoothSpeed(self.positioner.pose[0], x),
                           smoothSpeed(self.positioner.pose[1], y),
                           speed, smoothSpeed(self.positioner.pose[2], theta))
            waiting = wait

    def moveTo(self, x, y, theta, speed=1, tolerance=0.1, wait=0):
        # TODO: Implement Async?
        xDir = 0
        yDir = 0
        turnDir = 0
        waiting = 1
        while waiting:
            if not inTolerance(x, self.positioner.pose[0], tolerance):
                if x-self.positioner.pose[0] > 0:
                    xDir -= 1
                else:
                    xDir += 1
            if not inTolerance(y, self.positioner.pose[1], tolerance):
                if y-self.positioner.pose[1] > 0:
                    yDir -= 1
                else:
                    yDir += 1
            if not inTolerance(theta, self.positioner.pose[2], tolerance):
                if theta-self.positioner.pose[2] > 0:
                    turnDir -= 1
                else:
                    turnDir += 1
            self.cartesian(xDir, yDir, speed, turnDir)
            waiting = wait

    def comms(self):
        # This will most likely not work. Look for alternatives. Wiring everything to the pi isn't such a bad idea imo.
        while self.mecanum != 2:
            self.board.write(f"{self.lf},{self.rf},{self.lb},{self.rb}".encode("utf-8"))
            time.sleep(0.1)

    def drive(self, x, y, power, turn, flooring=False):
        if self.mecanum:
            return self.cartesian(x, y, power, turn, flooring)
        else:
            return self.tank(x, power, turn)

    def switchDrive(self):
        self.mecanum = not self.mecanum

    def brake(self):
        self.lf, self.rf, self.lb, self.rb = 0, 0, 0, 0

    def IK(self, x, y, theta):
        fl = (x - y - theta * ((self.geometry["x-center distance"]+self.geometry["y-center distance"])/2))

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


class LD06(Laser):

    def __init__(self, port="/dev/ttyUSB0"):
        Laser.__init__(self, 360, 10, 360, 12000, 0, 0)
        self.map = [0] * 360
        self.angles = []
        self.distances = []
        if sim:
            from numpy.random import randn
        else:
            self.loader, self.stop = _LD06(port)

    def read(self, return_angle=0):
        if sim:
            self.distances = randn(360)
            self.angles = list(range(360))
        else:
            self.angles, self.distances = self.loader['distances'].items()

        for angle, distance in zip(self.angles, self.distances):
            self.map[angle] = distance

        if return_angle:
            return self.distances, self.angles
        else:
            return self.map

    def exit(self):
        self.map = None
        if not sim:
            self.stop()


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
        self.pose = [0] * num_servos
        if not sim:
            self.kit = ServoKit(channels=8 if num_servos >= 8 else 16)
            self.arm = [self.kit.servo[i] for i in range(num_servos)]
        self.home = [90, 75, 130, 90, 150, 180]
        self.grabbing = [90, 10, 90, 100, 150, 180]
        self.dropping = [90, 50, 20, 0, 150, 0]
        self.move(self.home) if not sim else NotImplemented

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


def _LD06(port: str = '/dev/ttyUSB0') -> Tuple[dict, callable]:
    serial_port = Serial(port=port, baudrate=230400, timeout=5.0, bytesize=8, parity='N', stopbits=1)
    # distances are angles (in degrees) mapped to distance values (in centimeters)
    # last_packet_data is a LidarData instance containing the parsed data of the last packet
    data = {
        'distances': {},
        'last_packet_data': None
    }
    data_channel = {'interrupt': False}

    def update_data():
        # in bytes (speed and start_angle + 13 measurements + end_angle and timestamp + crc_check)
        packet_length = 4 + (3 * 12) + 4 + 1

        while not data_channel['interrupt']:
            last_byte_was_header = False
            buffer = ""

            # serial read loop
            # reads until it sees the header and var_len bytes, then parses the packet and resets the buffer
            while True:
                byte = serial_port.read()
                # convert byte to integer (big endian)
                byte_as_int = int.from_bytes(byte, 'big')

                # check for header byte
                if byte_as_int == 0x54:
                    buffer += byte.hex()
                    last_byte_was_header = True
                    # read next byte
                    continue

                # check for var_len byte (fixed value, comes after header byte)
                # if yes -> parse data, and update "data" variable
                if last_byte_was_header and byte_as_int == 0x2c:
                    buffer += byte.hex()

                    # if the packet length of the received packet doesn't have the expected length, something went wrong
                    # -> drop packet and restart read loop
                    if not len(buffer[0:-4]) == packet_length * 2:
                        buffer = ""
                        break

                    lidar_data = calc_lidar_data(buffer[0:-4])

                    # cleanup outdated values
                    for angle in lidar_data.angle_i:
                        start_angle, end_angle = lidar_data.start_angle, lidar_data.end_angle
                        # remove angle only if it is in the range of the current data packet
                        if angle in data['distances'] and (start_angle < angle < end_angle or (
                                end_angle > start_angle and (angle > start_angle or angle < end_angle))):
                            del data['distances'][angle]

                    # write new distance data to distances
                    for i, angle in enumerate(lidar_data.angle_i):
                        data['distances'][angle] = lidar_data.distance_i[i]

                    # override last_packet_data
                    data['last_packet_data'] = lidar_data

                    # reset buffer
                    buffer = ""
                    break
                else:  # is not header or var_len -> write to buffer
                    buffer += byte.hex()

                last_byte_was_header = False

    # call update_data in a separate thread to make listen_to_lidar return and update_data still update the "data" var
    read_thread = Thread(target=update_data)
    read_thread.start()

    def stop():
        # send interrupt signal
        data_channel['interrupt'] = True
        # wait for thread to terminate
        read_thread.join()
        serial_port.close()

    return data, stop


class LidarData:
    def __init__(self, start_angle, end_angle, crc_check, speed, time_stamp, confidence_i, angle_i, distance_i):
        self.start_angle = start_angle
        self.end_angle = end_angle
        self.crc_check = crc_check
        self.speed = speed
        self.time_stamp = time_stamp

        self.confidence_i = confidence_i
        self.angle_i = angle_i
        self.distance_i = distance_i


def calc_lidar_data(packet):
    # packet is a string representing a received packet in hexadecimal (1 byte / 8 bits := 2 characters)

    # retrieves nth byte of packet (as hexadecimal string; one character)
    # n can be negative to get nth last byte
    def get_byte(n: int) -> str:
        if n == -1:
            return packet[n * 2:]
        return packet[n * 2:(n + 1) * 2]

    # get infos from data packet (bytes are flipped for each value)
    # speed in degrees per second
    speed = int(get_byte(1) + get_byte(0), 16)
    # start and end angle in hundreds of a degree
    start_angle = float(int(get_byte(3) + get_byte(2), 16)) / 100  # degrees (after division)
    end_angle = float(int(get_byte(-4) + get_byte(-5), 16)) / 100  # degrees (after division)
    # timestamp in milliseconds (returns to 0 after 30000)
    time_stamp = int(get_byte(-2) + get_byte(-3), 16)
    crc_check = int(get_byte(-1), 16)

    confidence_i = list()
    angle_i = list()
    distance_i = list()

    # calculate the distance between (adjacent) measurements in degrees
    # the total travel of the motor (in degrees) divided by the amount of measurements (fixed at 12)
    if end_angle > start_angle:
        angle_step = float(end_angle - start_angle) / 12
    else:
        angle_step = float((end_angle + 360) - start_angle) / 12

    def circle(deg):
        return deg - 360 if deg >= 360 else deg

    # size of one measurement in bytes
    measurement_packet_size = 3
    # get data from each measurement (bytes are flipped for each value)
    # measurement data starts at the 5th byte (index: 4)
    for i in range(0, 12):
        # measurement position in measurement data section of packet
        measurement_position = measurement_packet_size * i
        # distance in millimeters
        distance_bytes = get_byte(5 + measurement_position) + get_byte(4 + measurement_position)
        distance_i.append(int(distance_bytes, 16) / 10)  # centimeters (after division)
        # intensity of signal (unitless)
        confidence_i.append(int(get_byte(6 + measurement_position), 16))
        # angle (position) of the measurement in degrees
        angle_i.append(circle(start_angle + (angle_step * i)))

    lidar_data = LidarData(start_angle, end_angle, crc_check, speed, time_stamp, confidence_i, angle_i, distance_i)
    return lidar_data


class Positioning:
    def __init__(self, starting=(0, 0, 0)):
        self.pose = starting
        self.encoders = None

    def getPose(self):
        return self.pose

    def updatePose(self, encoders):
        pass

    def setPose(self, pose):
        self.pose = pose

    def updateEncoders(self, readings):
        self.encoders = readings


class RP_A1(RPLidarA1):
    def __init__(self):
        super().__init__()
        self.lidar = RPLidar('/dev/ttyUSB0')
        print(self.lidar.get_info(), self.lidar.get_health())
        self.scanner = self.lidar.iter_scans()
        next(self.scanner)
        self.map = [0]*360

    def read(self):
        items = [item for item in next(self.scanner)]
        angles = [item[1] for item in items]
        distances = [item[2] for item in items]
        for i in range(len(angles)):
            self.map[angles[i]] = distances[i]
        return angles, distances

    def exit(self):
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()
