# Imports  (In descending length!!)
from __future__ import annotations
from pupil_apriltags import Detector as _d, Detection
from rplidar import RPLidar, RPLidarException
from serial.tools.list_ports import comports
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1
from adafruit_servokit import ServoKit
from evdev import InputDevice, ecodes
from _thread import interrupt_main
from dataclasses import dataclass
from operator import itemgetter
from datetime import datetime
from itertools import groupby
from pickle import dump, load
from threading import Thread
from time import sleep, time
from os import path
import numpy as np
import ikpy.chain
import struct
import serial
import math
import cv2


# Utilities
global_start = time()


class XboxController(object):
    MAX_TRIG_VAL = 1024
    MAX_JOY_VAL = 32768

    def __init__(self, deadzone=0.1, retries=5, stall=5):
        self.retries = retries
        self.stall = stall

        self.device_file = ""
        self.gamepad = None
        self.conn_t = None
        self.connect()
        self.deadzone = deadzone

        self.LJoyY = 0
        self.LJoyX = 0  # This. Also normalize joystick values
        self.RJoyY = 0
        self.RJoyX = 0
        self.LT = 0
        self.RT = 0
        self.LB = 0
        self.RB = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LJoyB = 0
        self.RJoyB = 0
        self.Back = 0
        self.Start = 0
        self.LD = 0
        self.RD = 0
        self.UD = 0
        self.DD = 0

        self._monitor_thread = Thread(target=self._monitor_controller, daemon=True)
        self._monitor_thread.start()

    def connect(self):
        existing = []
        names = ["Generic X-Box pad", "Xbox Wireless Controller", "Microsoft X-Box One S pad"]
        for i in range(50):
            if path.exists(f"/dev/input/event{i}"):
                existing.append(f"/dev/input/event{i}")
            if path.exists(f"/dev/input/js{i}"):
                existing.append(f"/dev/input/js{i}")
        for device in existing:
            try:
                gamepad = InputDevice(device)
                if gamepad.name in names:
                    self.device_file = device
                    self.conn_t = time()
                    self.gamepad = gamepad
                    break
            except OSError:
                continue
        else:
            raise OSError("No controller found")

    def read(self):  # return the buttons/triggers that you care about in this methode
        reads = [self.LJoyX, self.LJoyY, self.RJoyX, self.RJoyY,
                 self.RT, self.A, self.Back, self.Start]

        return [self._clean(i) for i in reads]

    def _clean(self, x):  # Filter out the inputs.
        return round(x, 3) if not self.deadzone > x > -self.deadzone else 0

    def _monitor_controller(self):
        from evdev import ecodes
        try:
            start = self.conn_t
            for event in self.gamepad.read_loop():
                if self.conn_t != start:
                    break
                if not path.exists(self.device_file):
                    print("Controller has disconnected. Trying to reconnect...")
                    # Indicates that the controller disconnected.
                    for i in range(self.retries):
                        try:
                            self.connect()
                            # Need to work on this, the reassignment of self.gamepad may break this loop.
                            self._monitor_thread = Thread(target=self._monitor_controller, daemon=True)
                            self._monitor_thread.start()
                            break
                        except OSError:
                            sleep(self.stall)
                    else:  # If the max amount of retries is reached
                        raise OSError("[ERROR] Controller: Could not reconnect to controller.")
                    continue


                # Axis
                if event.type == ecodes.EV_ABS:
                    if event.code == 1:
                        self.LJoyY = self._clean(-(event.value / XboxController.MAX_JOY_VAL) + 1)  # normalize between -1 and 1
                    elif event.code == 0:
                        self.LJoyX = self._clean(event.value / XboxController.MAX_JOY_VAL - 1)  # normalize between -1 and 1
                    elif event.code == 5:
                        self.RJoyY = self._clean(-(event.value / XboxController.MAX_JOY_VAL) + 1)  # normalize between -1 and 1
                    elif event.code == 2:
                        self.RJoyX = self._clean(event.value / XboxController.MAX_JOY_VAL) - 1  # normalize between -1 and 1
                    elif event.code == 10:
                        self.LT = self._clean(event.value / XboxController.MAX_TRIG_VAL)  # normalize between 0 and 1
                    elif event.code == 9:
                        self.RT = self._clean(event.value / XboxController.MAX_TRIG_VAL)  # normalize between 0 and 1
                        # DPad

                    elif event.code == 17:
                        self.UD = max(0, -event.value)
                        self.DD = max(0, event.value)

                    elif event.code == 16:
                        self.LD = max(0, -event.value)
                        self.RD = max(0, event.value)

                elif event.type == ecodes.EV_KEY:
                    # Bumpers
                    if event.code == 310:
                        self.LB = event.value
                    elif event.code == 311:
                        self.RB = event.value

                    # Face Buttons
                    elif event.code == 304:
                        self.A = event.value
                    elif event.code == 307:
                        self.X = event.value  # previously switched with X
                    elif event.code == 308:
                        self.Y = event.value  # previously switched with Y
                    elif event.code == 305:
                        self.B = event.value

                    # Joystick Buttons
                    elif event.code == 317:
                        self.LJoyB = event.value
                    elif event.code == 318:
                        self.RJoyB = event.value

                    # Menu Buttons
                    elif event.code == 314:
                        self.Back = event.value
                    elif event.code == 315:
                        self.Start = event.value

        except Exception as e:
            print(e)
            print("Controller has disconnected. Trying to reconnect...")
            # Indicates that the controller disconnected.
            for i in range(self.retries):
                try:
                    self.connect()
                    # Need to work on this, the reassignment of self.gamepad may break this loop.
                    self._monitor_thread = Thread(target=self._monitor_controller, daemon=True)
                    self._monitor_thread.start()
                    break
                except OSError:
                    sleep(self.stall)
            else:  # If the max amount of retries is reached
                raise OSError("[ERROR] Controller: Could not reconnect to controller.")

    @staticmethod
    def edge(pulse, last, rising=True):
        status = (pulse if rising else not pulse) and pulse != last
        return status, pulse

    def setTrigger(self, button_name, f, rising=True, polling=1/120, **kwargs):
        last = False
        def trigger(**kwargs):
            nonlocal last
            print(kwargs)
            while True:
                button = getattr(self, button_name)
                pulse, last = self.edge(button, last, rising)
                if pulse:
                    f(**kwargs)
                sleep(polling)
        Thread(target=trigger, kwargs=kwargs, daemon=True).start()
        print(f"Trigger set for {button_name} button.")


def find_port_by_vid_pid(vid, pid):
    ports = list(comports())

    for port in ports:
        print()
        if port.vid == vid and port.pid == pid:
            return port.device
    return None


def smoothSpeed(start, stop, lapse=100):
    def smooth(stamp):
        t = stamp/lapse
        if t < 0.5:
            res = 4 * t * t * t
        else:
            p = 2 * t - 2
            res = 0.5 * p * p * p + 1
        return stop*res + start*(1-res)

    return smooth


def getAngle(x, y):
    angle = round(math.atan2(y, x), 6)
    if angle < 0:
        angle += 2* math.pi
    return angle


def ecd(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.linalg.norm(a-b)


def inTolerance(a, b, tol=1):
    return abs(a - b) <= tol


def limit(x, low, high):
    return min(high, max(x, low))


def line2dots(line):
    x1, y1, x2, y2 = *line[0], *line[1]
    points = []
    issteep = abs(y2-y1) > abs(x2-x1)
    if issteep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    rev = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        rev = True
    deltax = x2 - x1
    deltay = abs(y2-y1)
    error = int(deltax / 2)
    y = y1
    ystep = None
    if y1 < y2:
        ystep = 1
    else:
        ystep = -1
    for x in range(x1, x2 + 1):
        if issteep:
            points.append((y, x))
        else:
            points.append((x, y))
        error -= deltay
        if error < 0:
            y += ystep
            error += deltax
    # Reverse the list if the coordinates were reversed
    if rev:
        points.reverse()
    return points


# Low-Level classes
class Rosmaster(object):
    __uart_state = 0
    CARTYPE_X3 = 0x01
    CARTYPE_X3_PLUS = 0x02
    CARTYPE_X1 = 0x04
    CARTYPE_R2 = 0x05
    VID = 0x1a86
    PID = 0x7523

    def __init__(self, car_type=0x02, com="/dev/ttyUSB0", delay=.002, debug=False):
        port = find_port_by_vid_pid(self.VID, self.PID)
        self.ser = serial.Serial(port, 115200)
        if not self.ser.isOpen():
            self.ser = serial.Serial(com, 115200)

        self.__delay_time = delay
        self.__debug = debug

        self.__HEAD = 0xFF
        self.__DEVICE_ID = 0xFC
        self.__COMPLEMENT = 257 - self.__DEVICE_ID
        self.__CAR_TYPE = car_type
        self.__CAR_ADJUST = 0x80

        self.FUNC_AUTO_REPORT = 0x01
        self.FUNC_BEEP = 0x02
        self.FUNC_PWM_SERVO = 0x03
        self.FUNC_PWM_SERVO_ALL = 0x04
        self.FUNC_RGB = 0x05
        self.FUNC_RGB_EFFECT = 0x06

        self.FUNC_REPORT_SPEED = 0x0A
        self.FUNC_REPORT_IMU_RAW = 0x0B
        self.FUNC_REPORT_IMU_ATT = 0x0C
        self.FUNC_REPORT_ENCODER = 0x0D

        self.FUNC_MOTOR = 0x10
        self.FUNC_CAR_RUN = 0x11
        self.FUNC_MOTION = 0x12
        self.FUNC_SET_MOTOR_PID = 0x13
        self.FUNC_SET_YAW_PID = 0x14
        self.FUNC_SET_CAR_TYPE = 0x15

        self.FUNC_UART_SERVO = 0x20
        self.FUNC_UART_SERVO_ID = 0x21
        self.FUNC_UART_SERVO_TORQUE = 0x22
        self.FUNC_ARM_CTRL = 0x23
        self.FUNC_ARM_OFFSET = 0x24

        self.FUNC_AKM_DEF_ANGLE = 0x30
        self.FUNC_AKM_STEER_ANGLE = 0x31

        self.FUNC_REQUEST_DATA = 0x50
        self.FUNC_VERSION = 0x51

        self.FUNC_RESET_FLASH = 0xA0

        self.__ax = 0
        self.__ay = 0
        self.__az = 0
        self.__gx = 0
        self.__gy = 0
        self.__gz = 0
        self.__mx = 0
        self.__my = 0
        self.__mz = 0
        self.__vx = 0
        self.__vy = 0
        self.__vz = 0

        self.__yaw = 0
        self.__roll = 0
        self.__pitch = 0

        self.encoders = [0, 0, 0, 0]
        self.enc_speed = [0, 0, 0, 0]
        self.prev_enc = [0, 0, 0, 0]
        self.last_update = time()

        self.__read_id = 0
        self.__read_val = 0

        self.__read_arm_ok = 0
        self.__read_arm = [-1, -1, -1, -1, -1, -1]

        self.__version_H = 0
        self.__version_L = 0
        self.__version = 0

        self.__pid_index = 0
        self.__kp1 = 0
        self.__ki1 = 0
        self.__kd1 = 0

        self.__arm_offset_state = 0
        self.__arm_offset_id = 0
        self.__arm_ctrl_enable = True

        self.__battery_voltage = 126.0

        self.__akm_def_angle = 100
        self.__akm_readed_angle = False
        self.__AKM_SERVO_ID = 0x01

        if self.__debug:
            print("cmd_delay=" + str(self.__delay_time) + "s")

        if self.ser.isOpen():
            print("Rosmaster Serial Opened! Baudrate=115200")
            self.set_car_type(car_type)
        else:
            print("Serial Open Failed!")
        sleep(.002)

    def __del__(self):
        self.ser.close()
        self.__uart_state = 0
        print("serial Close!")
        del self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.__del__()

    # According to the type of data frame to make the corresponding parsing
    def __parse_data(self, ext_type, ext_data):
        # print("parse_data:", ext_data, ext_type)
        if ext_type == self.FUNC_REPORT_SPEED:
            # print(ext_data)
            self.__vx = int(struct.unpack('h', bytearray(ext_data[0:2]))[0]) / 1000.0
            self.__vy = int(struct.unpack('h', bytearray(ext_data[2:4]))[0]) / 1000.0
            self.__vz = int(struct.unpack('h', bytearray(ext_data[4:6]))[0]) / 1000.0
            self.__battery_voltage = struct.unpack('B', bytearray(ext_data[6:7]))[0]
        # the original gyroscope, accelerometer, magnetometer data
        elif ext_type == self.FUNC_REPORT_IMU_RAW:
            gyro_ratio = 1 / 3754.9  # ±500dps
            self.__gx = struct.unpack('h', bytearray(ext_data[0:2]))[0] * gyro_ratio
            self.__gy = struct.unpack('h', bytearray(ext_data[2:4]))[0] * -gyro_ratio
            self.__gz = struct.unpack('h', bytearray(ext_data[4:6]))[0] * -gyro_ratio
            accel_ratio = 1 / 1671.84
            self.__ax = struct.unpack('h', bytearray(ext_data[6:8]))[0] * accel_ratio
            self.__ay = struct.unpack('h', bytearray(ext_data[8:10]))[0] * accel_ratio
            self.__az = struct.unpack('h', bytearray(ext_data[10:12]))[0] * accel_ratio
            mag_ratio = 1
            self.__mx = struct.unpack('h', bytearray(ext_data[12:14]))[0] * mag_ratio
            self.__my = struct.unpack('h', bytearray(ext_data[14:16]))[0] * mag_ratio
            self.__mz = struct.unpack('h', bytearray(ext_data[16:18]))[0] * mag_ratio
        # the attitude Angle of the board
        elif ext_type == self.FUNC_REPORT_IMU_ATT:
            self.__roll = struct.unpack('h', bytearray(ext_data[0:2]))[0] / 10000.0
            self.__pitch = struct.unpack('h', bytearray(ext_data[2:4]))[0] / 10000.0
            self.__yaw = struct.unpack('h', bytearray(ext_data[4:6]))[0] / 10000.0

        # Encoder data on all four wheels
        elif ext_type == self.FUNC_REPORT_ENCODER:
            self.prev_enc = self.encoders.copy()
            steps = list(range(0, 17, 4))
            self.encoders = [struct.unpack('i', bytearray(ext_data[steps[i]:steps[i+1]]))[0] for i in range(len(steps)-1)]
            timing = time()
            time_diff = timing-self.last_update
            self.last_update = timing
            for i in range(4):
                self.enc_speed[i] = (self.encoders[i]-self.prev_enc[i])/time_diff

        else:
            if ext_type == self.FUNC_UART_SERVO:
                self.__read_id = struct.unpack('B', bytearray(ext_data[0:1]))[0]
                self.__read_val = struct.unpack('h', bytearray(ext_data[1:3]))[0]
                if self.__debug:
                    print("FUNC_UART_SERVO:", self.__read_id, self.__read_val)

            elif ext_type == self.FUNC_ARM_CTRL:
                self.__read_arm[0] = struct.unpack('h', bytearray(ext_data[0:2]))[0]
                self.__read_arm[1] = struct.unpack('h', bytearray(ext_data[2:4]))[0]
                self.__read_arm[2] = struct.unpack('h', bytearray(ext_data[4:6]))[0]
                self.__read_arm[3] = struct.unpack('h', bytearray(ext_data[6:8]))[0]
                self.__read_arm[4] = struct.unpack('h', bytearray(ext_data[8:10]))[0]
                self.__read_arm[5] = struct.unpack('h', bytearray(ext_data[10:12]))[0]
                self.__read_arm_ok = 1
                if self.__debug:
                    print("FUNC_ARM_CTRL:", self.__read_arm)

            elif ext_type == self.FUNC_VERSION:
                self.__version_H = struct.unpack('B', bytearray(ext_data[0:1]))[0]
                self.__version_L = struct.unpack('B', bytearray(ext_data[1:2]))[0]
                if self.__debug:
                    print("FUNC_VERSION:", self.__version_H, self.__version_L)

            elif ext_type == self.FUNC_SET_MOTOR_PID:
                self.__pid_index = struct.unpack('B', bytearray(ext_data[0:1]))[0]
                self.__kp1 = struct.unpack('h', bytearray(ext_data[1:3]))[0]
                self.__ki1 = struct.unpack('h', bytearray(ext_data[3:5]))[0]
                self.__kd1 = struct.unpack('h', bytearray(ext_data[5:7]))[0]
                if self.__debug:
                    print("FUNC_SET_MOTOR_PID:", self.__pid_index, [self.__kp1, self.__ki1, self.__kd1])

            elif ext_type == self.FUNC_SET_YAW_PID:
                self.__pid_index = struct.unpack('B', bytearray(ext_data[0:1]))[0]
                self.__kp1 = struct.unpack('h', bytearray(ext_data[1:3]))[0]
                self.__ki1 = struct.unpack('h', bytearray(ext_data[3:5]))[0]
                self.__kd1 = struct.unpack('h', bytearray(ext_data[5:7]))[0]
                if self.__debug:
                    print("FUNC_SET_YAW_PID:", self.__pid_index, [self.__kp1, self.__ki1, self.__kd1])

            elif ext_type == self.FUNC_ARM_OFFSET:
                self.__arm_offset_id = struct.unpack('B', bytearray(ext_data[0:1]))[0]
                self.__arm_offset_state = struct.unpack('B', bytearray(ext_data[1:2]))[0]
                if self.__debug:
                    print("FUNC_ARM_OFFSET:", self.__arm_offset_id, self.__arm_offset_state)

    def __receive_data(self):
        while self.__uart_state:
            head1 = bytearray(self.ser.read())[0]
            if head1 == self.__HEAD:
                head2 = bytearray(self.ser.read())[0]
                check_sum = 0
                rx_check_num = 0
                if head2 == self.__DEVICE_ID - 1:
                    ext_len = bytearray(self.ser.read())[0]
                    ext_type = bytearray(self.ser.read())[0]
                    ext_data = []
                    check_sum = ext_len + ext_type
                    data_len = ext_len - 2
                    while len(ext_data) < data_len:
                        value = bytearray(self.ser.read())[0]
                        ext_data.append(value)
                        if len(ext_data) == data_len:
                            rx_check_num = value
                        else:
                            check_sum = check_sum + value
                    if check_sum % 256 == rx_check_num:
                        self.__parse_data(ext_type, ext_data)
                    else:
                        if self.__debug:
                            print("check sum error:", ext_len, ext_type, ext_data)

    # Request data, function: corresponding function word to return data, parm: parameter passed in
    def __request_data(self, function, param=0):
        cmd = [self.__HEAD, self.__DEVICE_ID, 0x05, self.FUNC_REQUEST_DATA, int(function) & 0xff, int(param) & 0xff]
        checksum = sum(cmd, self.__COMPLEMENT) & 0xff
        cmd.append(checksum)
        self.ser.write(cmd)
        if self.__debug:
            print("request:", cmd)
        sleep(0.002)

    # Arm converts Angle to position pulse
    @staticmethod
    def __arm_convert_value(s_id, s_angle):
        if 0 < s_id < 5 == 1:
            return int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 5:
            return int((3700 - 380) * (s_angle - 0) / (270 - 0) + 380)
        elif s_id == 6:
            return int((3100 - 900) * (s_angle - 0) / (180 - 0) + 900)
        else:
            return -1

    # Arm converts position pulses into angles
    @staticmethod
    def __arm_convert_angle(s_id, s_value):
        if 0 < s_id < 5:
            return int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 5:
            return int((270 - 0) * (s_value - 380) / (3700 - 380) + 0 + 0.5)
        elif s_id == 6:
            return int((180 - 0) * (s_value - 900) / (3100 - 900) + 0 + 0.5)
        else:
            return -1

    # Limit the PWM duty ratio value of motor input, value=127, keep the original data, do not modify the current motor speed
    def __limit_motor_value(self, value):
        if value == 127:
            return 127
        else:
            return limit(value, -100, 100)

    # Start the thread that receives and processes data
    def create_receive_threading(self):
        try:
            if self.__uart_state == 0:
                self.__uart_state = 1
                name1 = "task_serial_receive"
                task_receive = Thread(target=self.__receive_data, name=name1, daemon=True)
                task_receive.start()
                print("----------------create receive threading--------------")
        except Exception as e:
            print('---create_receive_threading error!---')
            print(e)
            pass

    # The MCU automatically returns the data status bit, which is enabled by default. If the switch is closed, the data reading function will be affected.
    # enable=True, The underlying expansion board sends four different packets of data every 10 milliseconds, so each packet is refreshed every 40 milliseconds.
    # If enable=False, the report is not sent.
    # forever=True for permanent, =False for temporary
    def set_auto_report_state(self, enable, forever=False):
        try:
            state1 = 1 if enable else 0
            state2 = 0x5f if forever else 0
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x05, self.FUNC_AUTO_REPORT, state1, state2]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("report:", cmd)
            sleep(self.__delay_time)
        except Exception as e:
            print('---set_auto_report_state error!---')
            print(e)
            pass

    # Buzzer switch. On_time =0: the buzzer is off. On_time =1: the buzzer keeps ringing
    # On_time >=10: automatically closes after xx milliseconds (on_time is a multiple of 10)
    def set_beep(self, on_time):
        try:
            if on_time < 0:
                print("beep input error!")
                return
            value = bytearray(struct.pack('h', int(on_time)))

            cmd = [self.__HEAD, self.__DEVICE_ID, 0x05, self.FUNC_BEEP, value[0], value[1]]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("beep:", cmd)
            sleep(self.__delay_time)
        except Exception as e:
            print('---set_beep error!---')
            print(e)
            pass

    # servo_id=[1, 4], angle=[0, 180]
    # Servo control, servo_id: corresponding, Angle: corresponding servo Angle value
    def set_pwm_servo(self, servo_id, angle):
        try:
            if not 1 <= servo_id <= 4:
                if self.__debug:
                    print("set_pwm_servo input invalid")
                return
            angle = limit(angle, 0, 180)
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_PWM_SERVO, int(servo_id), int(angle)]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("pwmServo:", cmd)
            sleep(self.__delay_time)
        except:
            print('---set_pwm_servo error!---')
            pass

    # At the same time control four PWM Angle, angle_sX=[0, 180]
    def set_pwm_servo_all(self, angle_s1, angle_s2, angle_s3, angle_s4):
        try:
            if angle_s1 < 0 or angle_s1 > 180:
                angle_s1 = 255
            if angle_s2 < 0 or angle_s2 > 180:
                angle_s2 = 255
            if angle_s3 < 0 or angle_s3 > 180:
                angle_s3 = 255
            if angle_s4 < 0 or angle_s4 > 180:
                angle_s4 = 255
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_PWM_SERVO_ALL,
                   int(angle_s1), int(angle_s2), int(angle_s3), int(angle_s4)]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("all Servo:", cmd)
            sleep(self.__delay_time)
        except:
            print('---set_pwm_servo_all error!---')
            pass

    # RGB control, can be controlled individually or collectively, before control need to stop THE RGB light effect.
    # Led_id =[0, 13], control the CORRESPONDING numbered RGB lights;  Led_id =0xFF, controls all lights.
    # Red,green,blue=[0, 255], indicating the RGB value of the color.
    def set_colorful_lamps(self, led_id, red, green, blue):
        try:
            id = int(led_id) & 0xff
            r = int(red) & 0xff
            g = int(green) & 0xff
            b = int(blue) & 0xff
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_RGB, id, r, g, b]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("rgb:", cmd)
            sleep(self.__delay_time)
        except:
            print('---set_colorful_lamps error!---')
            pass

    # RGB programmable light band special effects display.
    # Effect =[0, 6], 0: stop light effect, 1: running light, 2: running horse light, 3: breathing light, 4: gradient light, 5: starlight, 6: power display
    # Speed =[1, 10], the smaller the value, the faster the speed changes
    # Parm, left blank, as an additional argument.  Usage 1: The color of breathing lamp can be modified by the effect of breathing lamp [0, 6]
    def set_colorful_effect(self, effect, speed=255, parm=255):
        try:
            eff = int(effect) & 0xff
            spe = int(speed) & 0xff
            par = int(parm) & 0xff
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_RGB_EFFECT, eff, spe, par]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("rgb_effect:", cmd)
            sleep(self.__delay_time)
        except:
            print('---set_colorful_effect error!---')
            pass

    # Control PWM pulse of motor to control speed (speed measurement without encoder). speed_X=[-100, 100]
    # Note, we may or may not use the integrated pid controller for it, idk.
    def set_motor(self, speed_1=0, speed_2=0, speed_3=0, speed_4=0):
        try:
            t_speed_a = bytearray(struct.pack('b', self.__limit_motor_value(-speed_1)))
            t_speed_b = bytearray(struct.pack('b', self.__limit_motor_value(speed_2)))
            t_speed_c = bytearray(struct.pack('b', self.__limit_motor_value(-speed_3)))
            t_speed_d = bytearray(struct.pack('b', self.__limit_motor_value(speed_4)))
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_MOTOR,
                   t_speed_a[0], t_speed_b[0], t_speed_c[0], t_speed_d[0]]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("motor:", cmd)
            sleep(self.__delay_time)
        except IndexError as e:
            print('---set_motor error!---: ', e)
            pass

    # Clear the cache data automatically sent by the MCU
    def clear_auto_report_data(self):
        self.__battery_voltage = 0
        self.__vx, self.__vy, self.__vz = 0, 0, 0
        self.__ax, self.__ay, self.__az = 0, 0, 0
        self.__gx, self.__gy, self.__gz = 0, 0, 0
        self.__mx, self.__my, self.__mz = 0, 0, 0
        self.__yaw, self.__roll, self.__pitch = 0, 0, 0
        self.encoders = [0, 0, 0, 0]
        self.enc_speed = [0, 0, 0, 0]

    # Reset the car flash saved data, restore the factory default value
    def reset_flash_value(self):
        try:
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x04, self.FUNC_RESET_FLASH, 0x5F]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("flash:", cmd)
            sleep(self.__delay_time)
            sleep(.1)
        except:
            print('---reset_flash_value error!---')
            pass

    # Get accelerometer triaxial data, return a_x, a_y, a_z
    def get_accelerometer_data(self):
        return self.__ax, self.__ay, self.__az

    # Get the gyro triaxial data, return g_x, g_y, g_z
    def get_gyroscope_data(self):
        return self.__gx, self.__gy, self.__gz

    def get_magnetometer_data(self):
        return self.__mx, self.__my, self.__mz

    def get_imu_attitude_data(self, ToAngle=True):
        if ToAngle:
            RtA = 57.2957795
            return self.__roll * RtA, self.__pitch * RtA, self.__yaw * RtA
        else:
            return self.__roll, self.__pitch, self.__yaw

    # Get the battery voltage
    def get_battery_voltage(self):
        return self.__battery_voltage/10.0

    # Obtain data of four-channel motor encoder
    def get_motor_encoder(self):
        return self.encoders

    # Set car Type
    def set_car_type(self, car_type):
        if str(car_type).isdigit():
            self.__CAR_TYPE = int(car_type) & 0xff
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_SET_CAR_TYPE, self.__CAR_TYPE, 0x5F]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("car_type:", cmd)
            sleep(.1)
        else:
            print("set_car_type input invalid")


driver = Rosmaster(car_type=Rosmaster.CARTYPE_X3)


class Drive:  # TODO: Implement self.max properly
    max_speed = 1.0  # Measure this later.

    def __init__(self, max_speed=1, collision_fn=lambda x: [], arg=None):
        self.lf = 0
        self.rf = 0
        self.lb = 0
        self.rb = 0
        self.max = max_speed
        self.mecanum = 1
        self.collision_fn = collision_fn
        self.arg = arg
        self.thread_life = 1
        self.comm_thread = Thread(target=self.comms, daemon=True)
        self.comm_thread.start()
        self.test_speeds = ((1, 1, 1, 1), (-1, -1, -1, -1),  # Forward, Backward
                            (-1, 1, 1, -1), (1, -1, -1, 1),  # Left, Right
                            (0, 1, 1, 0), (-1, 0, 0, -1),    # TopLeft, BottomLeft
                            (1, 0, 0, 1), (0, -1, -1, 0),    # TopRight, BottomRight
                            (1, -1, 1, -1), (-1, 1, -1, 1))  # RotateRight, RotateLeft

    # Movement Functions
    def cartesian(self, x, y, speed=1, turn=0):
        theta = getAngle(y, x)
        #theta = rawTheta - math.pi / 2 if rawTheta > math.pi / 2 else 2 * math.pi - rawTheta
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

        clip = speed+abs(turn)

        if clip > 1:
            lf /= clip
            rf /= clip
            lb /= clip
            rb /= clip

        self.lf = lf
        self.rf = rf
        self.lb = lb
        self.rb = rb

        return lf, rf, lb, rb

    def tank(self, V, omega):
        """
        Convert differential drive commands to mecanum wheel speeds.

        Parameters:
        - V: Forward velocity (-1 to 1).
        - omega: Rotational velocity (-1 to 1), positive is clockwise.
        - width: Distance between left and right wheels. This is a scaling factor and doesn't need to be the actual width of the robot.

        Returns:
        Tuple containing normalized wheel speeds for (FL, FR, BL, BR).
        """

        V_FL_BL = V - omega / 2
        V_FR_BR = V + omega / 2

        # Find the maximum absolute value among all speeds
        max_speed = max(abs(V_FL_BL), abs(V_FR_BR))

        # If the max speed is greater than 1, we need to scale down all speeds
        if max_speed > 1:
            V_FL_BL /= max_speed
            V_FR_BR /= max_speed
        self.lf, self.rf, self.lb, self.rb = V_FL_BL, V_FR_BR, V_FL_BL, V_FR_BR
        return V_FL_BL, V_FR_BR, V_FL_BL, V_FR_BR

    # Note: For simulation, override this class to calculate the robot's position based on the motors. Also ticks.
    def comms(self, update_freq=10):
        while self.thread_life:
            driver.set_motor(round(self.lf*100), round(self.rf*100), round(self.lb*100), round(self.rb*100))
            sleep(1/update_freq)

    def drive(self, x, y, power, turn):
        if self.mecanum:
            return self.cartesian(x, y, power, turn)
        else:
            return self.tank(power, turn)

    def switchDrive(self):
        self.mecanum = not self.mecanum

    def brake(self):
        self.lf, self.rf, self.lb, self.rb = -self.lf, -self.rf, -self.lb, -self.rb
        sleep(0.1)
        self.lf, self.rf, self.lb, self.rb = 0, 0, 0, 0

    def smoothBrake(self, break_time=1):
        lfL = smoothSpeed(self.lf, 0)
        rfL = smoothSpeed(self.rf, 0)
        lbL = smoothSpeed(self.lb, 0)
        rbL = smoothSpeed(self.rb, 0)
        for i in range(101):
            self.lf, self.rf, self.lb, self.rb = lfL(i), rfL(i), lbL(i), rbL(i)
            sleep(break_time/100)

    def exit(self):
        self.brake()
        sleep(0.1)
        self.thread_life = 0

    def get_directions(self):
        return [
            (self.lf - self.rf - self.lb + self.rb)/4,
            (self.lf + self.rf + self.lb + self.rb)/4,
            (self.lf - self.rf + self.lb - self.rb)/4
        ]


class IMU:
    def __init__(self):
        self.mag = 0
        self.gyro = 0
        self.acc = 0

    def getMag(self):
        return driver.get_magnetometer_data()

    def getGyro(self):
        return driver.get_gyroscope_data()

    def getAccel(self):
        return driver.get_accelerometer_data()


class Arm:
    def __init__(self, num_servos=6, lapse=1, steps=100):
        try:
            self.chain = ikpy.chain.Chain.from_urdf_file('./arm.urdf',
                                                     active_links_mask=[bool(i%2) for i in range(10)])
        except FileNotFoundError:
            self.chain = ikpy.chain.Chain.from_urdf_file('../arm.urdf',
                                                         active_links_mask=[bool(i % 2) for i in range(10)])
        self.mask = [link.bounds[0] for link in self.chain.links]
        self.delay = lapse/steps
        self.steps = steps
        self.pose = np.array([0] * num_servos)
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
        sleep(period)
        self.grab()
        sleep(0.1)
        self.move(self.dropping)
        sleep(period)
        self.drop()
        sleep(0.1)
        self.move(self.home)

    def move(self, pose=None):
        if pose is None:
            pose = self.pose

        move_prof = [smoothSpeed(self.pose[i], pose[i], self.steps) for i in range(len(self.pose))]
        for i in range(self.steps):
            for n in range(len(self.arm)):
                self.arm[n].angle = int(move_prof[n](i))
            sleep(self.delay)

    def moveTo(self, target, vector=False):
        if vector:
            target = self.v2p(target)
        pose = self.chain2pose(self.chain.inverse_kinematics(target, initial_position=self.pose2chain(self.pose)))
        for i, angle in enumerate(pose[:5]):
            self.arm[i].angle = angle

    def fk(self):
        fk = self.chain.forward_kinematics()
        return fk[:3, 3], fk[:3, :3]

    def chain2pose(self, joints):
        return np.append(((joints-self.mask)*180.0/np.pi).round()[1::2], [self.pose[-1]])

    def pose2chain(self, pose=None):
        if pose is None:
            pose = self.pose.copy()

        return np.vstack(
            ([float('-inf')] * len(pose[:5]), pose[:5] * np.pi / 180)
        ).reshape(-1, order='F') + [link.bounds[0] for link in self.chain.links]

    @staticmethod
    def p2v(coords):
        d = np.linalg.norm(coords)
        azimuth = np.arctan2(coords[1], coords[0])
        elevation = np.arcsin(coords[2] / d)
        return d, azimuth, elevation

    @staticmethod
    def v2p(vec):
        d, a, e = vec
        x = d * np.cos(a) * np.cos(e)
        y = d * np.sin(a) * np.cos(e)
        z = d * np.sin(e)
        return x, y, z


class Battery:
    def __init__(self, threshold=11.1, polling_rate=10, custom_f=None):
        self.threshold = threshold
        self.polling = polling_rate
        if custom_f is None:
            self.thread = Thread(target=self.check_bat_voltage, daemon=True)
        else:
            self.thread = Thread(target=custom_f, daemon=True)
        self.thread.start()

    @staticmethod
    def get_voltage():
        return driver.get_battery_voltage()

    def check_bat_voltage(self):
        while True:
            voltage = self.get_voltage()
            if voltage <= self.threshold:
                print("[CRITICAL] Battery levels too low! Voltage: ", voltage)
                for i in range(5):
                    driver.set_beep(100)
                    sleep(0.2)
                interrupt_main()
                break
            sleep(self.polling)


class RP_A1(RPLidarA1):
    VID = 0x10c4
    PID = 0xea60

    def __init__(self, com="/dev/ttyUSB2", baudrate=115200, timeout=3, rotation=0, scan_type="normal", threaded=True):
        super().__init__()
        port = find_port_by_vid_pid(self.VID, self.PID)
        self.lidar = RPLidar(port, baudrate, timeout)
        print(self.lidar.get_info(), self.lidar.get_health())
        self.t = threaded
        self.lidar.clean_input()
        self.scanner = self.lidar.iter_scans(scan_type, False, 10)

        next(self.scanner)
        self.rotation = rotation % 360
        if self.t:
            self.latest = [[0], [0]]
            self.scans = Thread(target=self.threaded_read, daemon=True)
            self.scans.start()
        # last_scan = self.read()  # Return this for when we must clear the buffer if we don't read fast enough.

    def threaded_read(self):
        try:
            while self.t:
                self.latest = self.read()
        except RPLidarException as e:
            print(f"[ERROR] RPLidar: {e} | {e.args}")
            interrupt_main()

    def read(self, rotate=False):  # It's totally possible to move this to a thread.
        items = next(self.scanner)
        angles, distances = list(zip(*items))[1:]
        if rotate:
            distances, angles = list(zip(*self.rotate_lidar_readings(zip(distances, angles))))
        return [list(distances), list(angles)]

    def exit(self):
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()
        if self.t:
            self.t = False
            self.scans.join()

    def autoStopCollision(self, collision_threshold):
        # This returns only the clear angles.
        distance, angle = self.getScan()
        clear = []
        for i, dist in enumerate(distance):
            if dist > collision_threshold:
                clear.append(angle[i])
        return clear

    def getScan(self):
        return self.latest if self.t else self.read()

    def self_nav(self, collision_angles, collision_threshold, lMask=(180, 271), rMask=(270, 361)):
        self.map = [collision_threshold-1]*360
        distances, angles = self.getScan()
        for i, angle in enumerate(angles):
            self.map[int(angle)] = distances[i]

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


# Possible conflict here. The self.pose is different from the pose retrieved by the lidar, yet the same name.
# TODO: make a tick-to-lidar conversion thingamajig
# TODO: Actually, get the cart working and rework the entire kinematics system.
# The issue is directional permanence. The functions don't directly translate to position.
# One easy change is adding x += x_change * cos(theta) and y += y_change * sin(theta) to the kinematics.
# This basically makes the x and y axis change according to the current orientation of the bot.
# Actual testing is required.
class MecanumKinematics:  # units in centimeters.
    # lx, ly = 13.2, 8.5
    def __init__(self, radius=10, wheel2centerDistance=21.7):
        self.tpr = 362  # Output revolutions (rpm=333.333)
        self.r = radius
        self.revs = [0, 0, 0, 0]  # How many times every wheel has turned. Good luck with that.
        self.w2c = wheel2centerDistance
        # these take the amount of rotations per wheel and return the movement in centimeters.
        self.vec = (0, 0)
        self.pose = [0, 0, 0]
        self.prev_ticks = [0, 0, 0, 0]
        self.x = lambda lf, rf, lb, rb: (lf - rf - lb + rb) * (self.r / 4) * np.cos(self.pose[2])
        self.y = lambda lf, rf, lb, rb: (lf + rf + lb + rb) * (self.r / 4) * np.sin(self.pose[2])
        self.w = lambda lf, rf, lb, rb: (lf - rf + lb - rb) * (self.r / (4 * self.w2c))

        self.timestampSecondsPrev = None
        self.thread = Thread(target=self.updatePosition, daemon=True)
        self.thread.start()

    def updatePosition(self):
        while True:
            self.revs = [i/self.tpr for i in driver.encoders]  # this turns the pose into revolution-based
            # This means that 1 turn on a wheel is 10cm of distance. Therefore,
            self.pose[0] = self.x(*self.revs)
            self.pose[1] = self.y(*self.revs)
            self.pose[2] = self.w(*self.revs)
            self.vec = (math.hypot(*self.pose[:2]), math.atan2(*self.pose[1::-1]))
            sleep(1 / 30)

    def getTicks(self, x, y, w):  # This already has the right hand rule coordinate system
        recip = 1 / self.r
        turn = self.w2c * w
        xy = x + y
        notxy = x - y
        return [recip * (notxy - turn), recip * (xy + turn), recip * (xy - turn), recip * (notxy + turn)]

    def computePoseChange(self, timestamp=None):
        timestamp = time() - global_start if timestamp is None else timestamp
        dXMillimeters = 0
        dYMillimeters = 0
        dthetaDegrees = 0
        dtSeconds = 0

        # Extract odometry
        FL, FR, RL, RR = [self.revs[i] - self.prev_ticks[i] for i in range(4)]
        self.prev_ticks = self.revs.copy()

        if self.timestampSecondsPrev is not None:

            # Compute motion components based on mecanum equations
            dXMillimeters = self.x(FL, FR, RL, RR)
            dYMillimeters = self.y(FL, FR, RL, RR)
            dthetaDegrees = self.w(FL, FR, RL, RR)

            dtSeconds = timestamp - self.timestampSecondsPrev

        # Store current timestamp for next time
        self.timestampSecondsPrev = timestamp

        dxyMillimeters = (dXMillimeters ** 2 + dYMillimeters ** 2) ** 0.5  # Total distance in XY plane

        # Return dXY, dtheta, and dt
        return dxyMillimeters, dthetaDegrees, dtSeconds


# Navigation
def pymap(n, func):
    return map(func, n)


class Map:
    paths = []

    def __init__(self, map, map_meters=35):
        # The IR Map is just the RRT Map format.
        self.map_meters = None
        if isinstance(map, str):
            if map.lower() == "random":
                # Get a randomly generated map for testing.
                try:
                    self.map = np.load("./map.npy")  # Formatted as an RRT Map.
                except FileNotFoundError:
                    self.map = np.load("../map.npy")
                self.map_meters = 35
            elif "pkl" in map:
                with open(map, "rb") as f:
                    values = load(f)
                if len(values) == 2 and (isinstance(values, tuple) or isinstance(values, list)):
                    meters, bytearr = values
                    map = np.array(bytearr).reshape(int(len(bytearr) ** 0.5), int(len(bytearr) ** 0.5))
                    for index in np.argwhere(map < 200):  # First we pull down the readings below quality.
                        map[index[0], index[1]] = 0
                    for index in np.argwhere(
                            map >= 200):  # Then we pull up the readings above quality and convert to 1.
                        map[index[0], index[1]] = 1
                    # Finally we invert the map, so 0 is free space and 1 is occupied space.
                    self.map = np.logical_not(map).astype(int)
                    self.map_meters = meters
                else:
                    self.__init__(values)
            else:
                raise ValueError("Could not extract map from .pkl file.")

        elif isinstance(map, bytearray):
            # convert from slam to IR
            # Slam maps are bytearrays that represent the SLAM Calculated map. Higher the pixel value, clearer it is.
            map = np.array(map).reshape(int(len(map) ** 0.5), int(len(map) ** 0.5))
            for index in np.argwhere(map < 200):  # First we pull down the readings below quality.
                map[index[0], index[1]] = 0
            for index in np.argwhere(map >= 200):  # Then we pull up the readings above quality and convert to 1.
                map[index[0], index[1]] = 1
            # Finally we invert the map, so 0 is free space and 1 is occupied space.
            self.map = np.logical_not(map).astype(int)

        elif isinstance(map, np.ndarray):
            # convert from rrt to IR
            # RRT Maps are numpy arrays that RRT uses to calculate a route to a point. 1 represents an obstacle.
            if np.max(map) > 1:
                for index in np.argwhere(map < 200):  # First we pull down the readings below quality.
                    map[index[0], index[1]] = 0
                for index in np.argwhere(map >= 200):  # Then we pull up the readings above quality and convert to 1.
                    map[index[0], index[1]] = 1
                # Finally we invert the map, so 0 is free space and 1 is occupied space.
                self.map = np.logical_not(map).astype(int)
            else:  # If we get an IR Map, we just use it.
                self.map = map

        elif isinstance(map, int):
            # generate a blank IR Map
            self.map = np.zeros((map, map), dtype=int)

        else:
            raise ValueError("Map format unidentified.")
        self.map_center = [i//2 for i in self.map.shape]
        if self.map_meters is None:
            self.map_meters = map_meters

    def update(self, map):
        self.__init__(map)

    def fromSlam(self, map):
        size = int(len(map) ** 0.5)  # get the shape for a square map (1d to 2d)
        # convert from bytearray to 2d np array, apply quality threshold, scale down to 0-1, reshape
        map = ((np.array(map) - 73) / 255).reshape(size, size).round()
        self.map = np.logical_not(map).astype(int)

    def toSlam(self):
        return bytearray([(not i)*255 for i in self.map.flatten()])

    def toUnity(self):
        return bytearray(self.map.astype(np.uint8).flatten())

    def save(self, name=None):
        if name is None:
            dt = datetime.today()
            name = f"{dt.day}-{dt.month}-{dt.year} {dt.hour}:{dt.minute}.pkl"
        with open(name, "wb") as f:
            dump(self.map, f)
        print(f"[INFO] Saved Map as {name}!")

    def isValidPoint(self, point):
        return not self.map[point[0], point[1]]

    def getValidPoint(self) -> tuple:
        free = np.argwhere(self.map == 0)
        return tuple(free[np.random.randint(0, free.shape[0])])

    def __len__(self):
        return len(self.map)

    def tocv2(self, invert=True):
        map = self.map if not invert else np.logical_not(self.map)
        return cv2.cvtColor(map.astype(np.uint8) * 255, cv2.COLOR_GRAY2BGR)

    def drawPoint(self, img, point, r=2, c=(0, 0, 255), t=2):
        return cv2.circle(img, (point[0], self.map.shape[0]-point[1]), r, c, t)

    def drawPx(self, img, point, c=(0, 0, 255), r=1):
        for a in range(-r, r):
            for b in range(-r, r):
                img[self.map.shape[0]-(point[1]+b)][point[0]+a] = c
        return img

    def drawLine(self, img, line, c=(0, 255, 0), **kwargs):
        return cv2.line(img, (line[0][0], self.map.shape[0]-line[0][1]),
                        (line[1][0], self.map.shape[0]-line[1][1]), c, **kwargs)

    def drawLineOfDots(self, img, line, c=(0, 255, 0)):
        [self.drawLine(img, (line[i], line[i+1]), c=c, thickness=2) for i in range(len(line)-1)]

    def getValidRoute(self, n):
        return [self.getValidPoint() for _ in range(n)]

    def _posearrowext(self, pose, r):
        return pose[:2], (round(pose[0]+r*np.cos(pose[2]+3.14159)), round(pose[1]+r*np.sin(pose[2]+3.14159)))

    def animate(self, img=None, pose=None, drawLines=True, arrowLength=20, thickness=5):
        # Pose is expected to be 2 coordinates, which represent a center and a point along a circle.
        if img is None:
            #self.map = np.rot90(self.map, 3)
            img = self.tocv2()  # Why??? TODO: Make it so that the fucking map WORKS.

            # Restoring the map
            #self.map = np.rot90(self.map)
        if drawLines:
            for path in self.paths:
                try:
                    path = path.tolist()
                except AttributeError:
                    pass
                if path:
                    img = self.drawPoint(img, path[0][0], 4)
                    img = self.drawPoint(img, path[-1][1], 4)
                    color = np.random.randint(0, 256, 3).tolist()
                    for line in path:
                        img = self.drawLine(img, line, color)
        if pose is not None:
            if pose == "center":
                cv2.arrowedLine(img, self.map_center, tuple(pymap(self.map_center, lambda x: x-5)), (0, 0, 255), 3)
            else:
                pt1, pt2 = self._posearrowext(pose, arrowLength)
                cv2.arrowedLine(img, pt1, pt2, (0, 0, 255), thickness)
        cv2.imshow("Map", img)
        cv2.waitKey(1)

    def addPath(self, route: np.ndarray):
        try:
            if route is None:
                return
            int(route[0][0][0])  # check if we can index that deep and the value is a number
            # If that happens, we are sure it's a path
            self.paths.append(route)
        except TypeError:  # If we could not convert to an integer,
            [self.paths.append(i) for i in route]  # It means that route[0][0][0] was an array.
        except IndexError:  # If the probe was not successful, it's invalid.
            print("Empty or Invalid path provided.")

    def collision_free(self, a, b) -> bool:
        # Bresenham's line algorithm
        y0, x0 = int(a[0]), int(a[1])
        y1, x1 = int(b[0]), int(b[1])
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy

        while True:
            if x0 < 0 or x0 >= self.map.shape[0] or y0 < 0 or y0 >= self.map.shape[1]:
                break  # Out of bounds
            if self.map[y0, x0] != 0:  # Note the y0, x0 order for numpy arrays
                return False
            if x0 == x1 and y0 == y1:
                return True
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
        return False  # Return False if out of bounds


class SLAM:
    def __init__(self, lidar=None, map_handle=None, update_map=1):
        self.map = map_handle if map_handle else Map(800)
        if lidar is None:
            lidar = RPLidarA1()
        self.mapbytes = self.map.toSlam()
        self.ShouldUpdate = update_map
        self.map_size = len(self.map)  # Ensure it's actually an integer, might've been calculated.
        self.pose = (0, 0, 0)
        # Slam Preparation
        self.ratio = self.map.map_meters*1000 / self.map_size  # For converting MM to px.
        self.slam = RMHC_SLAM(lidar, self.map_size, self.map.map_meters, map_quality = 50)
        self.slam.setmap(self.mapbytes)

    def update(self, distances, angles, odometry=None):
        if angles is None:
            raise ValueError("Angles are required for SLAM. The array method has been deprecated.")
        self.slam.update(distances, odometry, angles, self.ShouldUpdate)
        # keep in mind that SLAM basically remains running on its own map. So no need to worry about data loss
        if self.ShouldUpdate:
            self.slam.getmap(self.mapbytes)  # as seen here, SLAMOps run on self.mapbytes exclusively.
            self.map.fromSlam(self.mapbytes)  # then, it gets exported to self.map
        self.pose = self.slam.getpos()
        return self.pose2px(self.pose)  # SLAM

    def pose2px(self, pose=None):
        return round(pose[0] / self.ratio), round(pose[1] / self.ratio), pose[2]  # theta is just rotation so it's fine

    def px2pose(self, px=None):
        return px[0] * self.ratio, px[1] * self.ratio, px[2] if len(px) == 3 else None

    def pose2cv2(self, size=10):  # TF was i tryna do?
        x, y, r = self.pose  # in px
        return (x, y), (int(x + size * np.cos(r)), int(y + size * np.sin(r)))


@dataclass
class Node:
    x: float
    y: float
    parent: 'Node' = None

    def __getitem__(self, item):
        return (self.x, self.y)[item]


class RRT:
    def __init__(self, map, step_size=25, iterations=1000):
        self.map = map
        self.step_size = step_size
        self.iterations = iterations

    def plan(self, start, end):
        start, end = np.array(start), np.array(end)
        # If you can go straight to the goal, do it
        if self.map.collision_free(start, end):
            return [[start.tolist(), end.tolist()]]

        start_node = Node(*start)
        goal_node = Node(*end)

        # Set up 2 trees to expand
        tree_a = [start_node]
        tree_b = [goal_node]

        for _ in range(self.iterations):
            # Pick a random point in the map to explore
            # Oportunity: Reduce the randomness, to make certain-er paths
            # Maybe reduce the options to an ellipse between both points
            random_point = Node(np.random.uniform(0, self.map.map.shape[0]), np.random.uniform(0, self.map.map.shape[1]))

            # Extend tree A towards the random point
            nearest_node_a = self._nearest(tree_a, random_point)
            new_node_a = self._steer(nearest_node_a, random_point)
            if new_node_a:
                tree_a.append(new_node_a)

                # Try to connect new node in tree A to nearest node in tree B
                nearest_node_b = self._nearest(tree_b, new_node_a)
                new_node_b = self._steer(nearest_node_b, new_node_a)
                while new_node_b:
                    tree_b.append(new_node_b)
                    if new_node_a.x == new_node_b.x and new_node_a.y == new_node_b.y:  # Trees are connected
                        # Path found
                        # Combine and reverse path from goal to start
                        out = self._rearrange(self.extract_path(new_node_a) + self.extract_path(new_node_b)[::-1], start, end)
                        # Here goes all the line postprocessing fluff

                        if out[-1][1] != end.tolist():
                            out[-1] = out[-1][::-1]
                        return out

                    nearest_node_b = self._nearest(tree_b, new_node_a)
                    new_node_b = self._steer(nearest_node_b, new_node_a)

            # Swap trees
            tree_a, tree_b = tree_b, tree_a

    def _steer(self, from_node, to_node):
        # Steering is picking a new step rate for each direction (so basically an angle abstracted into sine and cosine)
        direction = np.array([to_node.x - from_node.x, to_node.y - from_node.y])  # get the distance
        norm = np.linalg.norm(direction)  # hypotenuse
        if norm == 0:
            return None  # from_node and to_node are the same

        # Get the direction in interval I = [-1, 1], and make a step via the step size or the norm, whichever is smaller.
        direction = direction / norm * min(self.step_size, norm)

        # Step towards the desired direction.
        new_node = Node(from_node.x + direction[0], from_node.y + direction[1], parent=from_node)
        if self.map.collision_free(from_node, new_node):
            return new_node
        return None

    @staticmethod
    def _nearest(nodes, target):
        # check the closest node to the target
        distances = [np.hypot(node.x - target.x, node.y - target.y) for node in nodes]
        nearest_index = np.argmin(distances)
        return nodes[nearest_index]

    def _rearrange(self, path, start, end):
        # Check if the path is inverse.

        # First, if the end point is at the start
        if np.argwhere(np.all(path[0] == end, axis=-1)).size != 0:
            path.reverse()

        # Second, if the start point is at the end
        elif np.argwhere(np.all(path[-1] == start, axis=-1)).size != 0:
            raise ValueError(f"Inverted at second check.")

        # This is a continuity checker, i think?
        # Yes, this loop stops when i is a broken bond.
        last = path[0][1]
        for i in range(1, len(path) - 1):
            segment = path[i]
            if last != segment[0]:
                break
            last = segment[1]
        else:  # The else runs when the for loop completes.
            return path

        a, b = path[:i], path[i:]  # Split the path into the convergence of both trees
        b = np.fliplr(b).tolist()  # flip the second tree
        out = a + b  # Join them back properly
        return self.restitch(out)
        # This is a continuity checker. It tries to patch holes in the path by connecting disjointed segments.

    @staticmethod
    def calculate_slope(p1, p2):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return dy / dx if dx != 0 else None  # Handle vertical lines

    @staticmethod
    def slopes_are_close(slope1, slope2, tolerance=0.1):
        # not necesarily slopes, just comparing two numbers
        if slope1 is None and slope2 is None:
            return True
        if slope1 is None or slope2 is None:
            return False
        return abs(slope1 - slope2) <= tolerance

    def extract_path(self, end_node, slope_tolerance=1e-1):
        # This traverses the tree and extracts the xy location of each node.
        # After that, it simplifies the path by removing unnecessary points, and making line segments.

        points = []
        current_node = end_node
        while current_node is not None:
            points.append([current_node.x, current_node.y])
            current_node = current_node.parent

        points = np.array(points[::-1])  # Reverse to start from the beginning of the path

        if len(points) < 2:
            return []
        # This is the simplifier. It takes a list of points and returns a list of segments.
        segments = []
        current_slope = self.calculate_slope(points[0], points[1])
        start_point = points[0]

        for i in range(1, len(points)):
            new_slope = self.calculate_slope(points[i - 1], points[i]) if i < len(points) - 1 else None
            if not self.slopes_are_close(current_slope, new_slope, slope_tolerance) or i == len(points) - 1:
                # Before finalizing the segment, check for collisions along the proposed segment
                if self.map.collision_free(start_point, points[i]):
                    # If no collision, finalize the segment
                    segments.append([start_point.astype(int).tolist(), points[i].astype(int).tolist()])
                    start_point = points[i]
                    current_slope = new_slope
                else:
                    # If there's a collision, break the segment at the last collision-free point
                    # and start a new segment from there
                    segments.append([start_point.astype(int).tolist(), points[i - 1].astype(int).tolist()])
                    start_point = points[i - 1]
                    current_slope = self.calculate_slope(points[i - 1], points[i])

        return segments

    # Path Postprocessing methods
    # Check this, it is probably very broken.
    def ecd_shortening(self, path, start, end):
        # This function clips the ends of the path to the nearest collision-free point.
        if len(path) == 1:
            return path
        print(f"Shortening path...")
        furthest_start = 0
        soonest_end = None
        map = self.map

        # Find the closest indices where the start and end points can be connected to the path directly
        for i, line in enumerate(path):
            if map.collision_free(line[0], start):
                furthest_start = i
            if soonest_end is None and map.collision_free(line[0], end):
                soonest_end = i

        new = [[start.tolist(), path[furthest_start][0]]] + path[furthest_start:soonest_end]
        if path[soonest_end][0] == end.tolist():
            print("End path will repeat")
            print(new + [[path[soonest_end][0], end.tolist()]])
            new.append([path[soonest_end - 1][1], end.tolist()])
            print(new)
        else:
            new += [[path[soonest_end][0], end.tolist()]]
        # Replace all previous segments for the shortened segments.
        print(f"Final shortening: {new}")
        return new

    @staticmethod
    def restitch_end(path):  # This line end stitching happens when the final segment is the end point repeated.
        if len(path) == 1:
            return path
        # This can lead to a serious mistake.
        end, prev = path[-1], path[-2]
        if prev[1] != end[0]:
            print(f"Stitching end: {path}")
            return path[:-1] + [[prev[1], end[0]]] + [path[-1]]
        return path

    @staticmethod
    def check_continuity(path):
        prev = path[0][1]
        for i in range(1, len(path)):
            segment = path[i]
            if prev != segment[0]:
                return False
            prev = segment[1]
        return True

    @staticmethod
    def restitch(path):
        out = path
        while True:
            last = out[0][1]  # Get first segment
            for i in range(1, len(out)):  # For every segment
                segment = out[i]  # get the segment
                if last != segment[0]:  # If the last point of the last segment is not the first point of this segment
                    a, b = out[:i], out[i:]
                    out = a + [[a[-1][1], b[0][0]]] + b  # Make a bridge between both segments
                    break  # Try the continuity check again
                last = segment[1]  # If not, move on to the next segment
            else:
                return out  # If the continuity check passes, return the path

    @staticmethod
    def getEndPoints(lines):
        lines = np.array(lines)
        return np.append([lines[0][0]], lines[:, 1], 0)

    @staticmethod
    def endpointsToPath(endpoints):
        return np.array([[endpoints[i], endpoints[i + 1]] for i in range(len(endpoints) - 1)])

    @staticmethod
    def isValidPath(path):
        return (path is not None) and len(path) > 0


# Detection
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


def draw_cross(img, center, angle, length, color=(0, 0, 255), thickness=2):
    l1_x = length * np.cos(angle)
    l1_y = length * np.sin(angle)
    l2_x = length * np.cos(angle + np.pi/2)
    l2_y = length * np.sin(angle + np.pi/2)

    l1_start = (int(center[0] - l1_x),
                int(center[1] - l1_y))
    l1_end = (int(center[0] + l1_x),
                int(center[1] + l1_y))
    l2_start = (int(center[0] - l2_x),
                int(center[1] - l2_y))
    l2_end = (int(center[0] + l2_x),
                int(center[1] + l2_y))

    cv2.line(img, l1_start, l1_end, color, thickness)
    cv2.line(img, l2_start, l2_end, color, thickness)


def draw_arrow(img, center, angle, length, color=(0, 0, 255), thickness=2):
    l1_x = length * np.cos(angle)
    l1_y = length * np.sin(angle)

    start = (int(center[0] - l1_x),
                int(center[1] - l1_y))
    end = (int(center[0] + l1_x),
                int(center[1] + l1_y))

    cv2.arrowedLine(img, start, end, color, thickness)


class ApriltagDetector(_d):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def detect(self, image, estimate_tag_pose=False, camera_params=None, tag_size=None):
        detections = super().detect(image, estimate_tag_pose, camera_params, tag_size)
        return [Tag(detection) for detection in detections]


class Tag(Detection):
    def __init__(self, tag):
        super().__init__()
        self.corners = tag.corners.astype(int)[::-1]
        self.id = tag.tag_id
        self.center = tag.center.astype(int)
        self.pose_R = tag.pose_R
        self.pose_t = tag.pose_t
        self.homography = tag.homography
        self.decision_margin = tag.decision_margin
        self._r = np.array([0, 0, 0])
        self.hamming = tag.hamming
        self.family = tag.tag_family

    def draw(self, img, center=False, lockon=False, outline=False):
        if outline:
            cv2.polylines(img, [self.corners], True, (0, 255, 0), 2)
        if lockon:
            draw_cross(img, self.corners[0], self.rotation[2]-np.pi/2, 20, (0, 0, 0), 2)
            draw_cross(img, self.corners[1], self.rotation[2]-np.pi/2, 20, (0, 255, 0), 2)
            draw_cross(img, self.corners[2], self.rotation[2]-np.pi/2, 20, (255, 255, 0), 2)
            draw_cross(img, self.corners[3], self.rotation[2]-np.pi/2, 20, (255, 0, 0), 2)
        else:
            cv2.drawMarker(img, self.corners[0], (0, 0, 0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
            cv2.drawMarker(img, self.corners[1], (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
            cv2.drawMarker(img, self.corners[2], (255, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
            cv2.drawMarker(img, self.corners[3], (255, 0, 0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
        if center:
            draw_arrow(img, self.center, self.rotation[2]-np.pi/2, 20, (0, 0, 255), 2)
            draw_arrow(img, self.corners[0], self.rotation[1]-np.pi/2, 20, (255, 0, 255), 2)
            draw_arrow(img, self.corners[1], self.rotation[0]-np.pi/2, 20, (255, 0, 255), 2)


    def get_north(self):
        a, b = self.corners[:2]
        # Center of this line
        c = (a + b) // 2
        # The angle
        top_of_the_frame = np.array([c[0], 0])
        angle = np.arctan2(top_of_the_frame[1] - c[1], top_of_the_frame[0] - c[0])
        return c, angle


    def __str__(self):
        return f"Tag {self.id} at {self.center} with pose {self.pose_t.round(4).tolist()} and rotation {self.rotation.round(4).tolist()}"

    @property
    def rotation(self):  # Computing Euler Angles from a rotation matrix, Gregory G. Slabaugh
        # Remember that the paper uses 1-start indexing
        x, y, z = 0, 0, 0  # epsilon, theta, phi
        if abs(self.pose_R[2, 0]) != 1:
            y = -np.arcsin(self.pose_R[2, 0])
            x = np.arctan2(self.pose_R[2, 1]/np.cos(y), self.pose_R[2, 2]/np.cos(y))
            z = np.arctan2(self.pose_R[1, 0]/np.cos(y), self.pose_R[0, 0]/np.cos(y))
        else:  # This solves gimbal lock.
            z = 0
            r31_sign = -self.pose_R[2, 0]  # If abs == 1, we can use it directly to convert. Also, negate the -sin.
            y = r31_sign*np.pi/2
            x = np.arctan2(r31_sign*self.pose_R[0, 1], r31_sign*self.pose_R[0, 2])
        self._r = np.array([x, y, z])
        self._r[self._r < 0] += 2*np.pi

        return self._r

    @staticmethod
    def wrapper(vals):
        vals = np.array(vals)
        vals[vals < 0] += 2*np.pi
        return vals

    def general_solution(self, order: list):
        # Numerical Axis Assignment
        mapping = {"x": 0, "y": 1, "z": 2}
        # Get the order in independent variables, for ease
        r1, r2, r3 = list(map(lambda x: mapping[x], order))

        # Figure out the sign for the sine operation
        sign_map = {
            ("x", "y", "z"): 1,
            ("y", "z", "x"): 1,
            ("z", "x", "y"): 1,

            ("x", "z", "y"): -1,
            ("y", "x", "z"): -1,
            ("z", "y", "x"): -1,
        }
        key = tuple(order)
        sign = sign_map[key]

        # Deducing rotation angles
        if abs(self.pose_R[r1, r3]) != 1:
            second_rotation = sign * np.arcsin(self.pose_R[r1, r3])
            first_rotation = np.arctan2(-sign * self.pose_R[r2, r3]/np.cos(second_rotation), self.pose_R[r3, r3]/np.cos(second_rotation))
            third_rotation = np.arctan2(-sign * self.pose_R[r1, r2]/np.cos(second_rotation), self.pose_R[r1, r1]/np.cos(second_rotation))
        else:
            raise ValueError("Gimbal Lock is not handled in this function.")  # TODO: Implement Gimbal Lock

        # Sorting into position
        rotations = [0] * 3
        rotations[r1] = first_rotation
        rotations[r2] = second_rotation
        rotations[r3] = third_rotation
        return self.wrapper(rotations)

    @rotation.setter
    def rotation(self, x):
        raise AttributeError("Rotation is a read-only property.")

# made it B)


if __name__ == "__main__":
    import atexit
    # TODO: Add lidar scan quality filtering
    # aka if the scan is pretty bad, don't use it.
    # Also sync the lidar with the slam algo so that we can be more efficient
    # And still run the main code at a high framerate.
    class Robot:
        def __init__(self, map_type=800, cam_src=0, lidar_thread=True, lidar_rotation=0):
            # low-level stuff
            atexit.register(self.exit)
            self.rotate_reading = lidar_rotation != 0
            self.lt = lidar_thread
            self.driver = driver
            self.drive = Drive()
            self.lidar = RP_A1(rotation=lidar_rotation, threaded=self.lt)
            self.mpu = IMU()
            # self.arm = Arm()  # TODO: FIX THIS IN HARDWARE FFS
            self.cam = Camera(cam_src)
            self.kine = MecanumKinematics()  # TODO: add the proper parameters.

            # algos
            self.map = Map(map_type)
            # Only update if it's a new map.
            self.slam = SLAM(self.lidar, self.map, isinstance(map_type, int))
            self.planner = RRT(self.map)

            # info
            self.slam_pose = [0, 0, 0]
            self.kine_pose = self.kine.pose  # TODO: Test this. It may be a pointer, it may not, idk.
            self.pose = [0, 0, 0]  # TODO: figure out a way to mix both (IF they are both functional)
            self.mode = "manual"
            self.target = None
            self.path = None

        # concepts for hardware and algo updates, just a possibility.
        def _llu(self):
            pass

        def _hlu(self):
            pass

        def update(self):
            if self.mode == "manual":
                pass
            elif self.mode == "auto":
                pass

            # Constant functions:

        def auto(self):
            # Path planning stuff
            if self.target is not None:
                self.drive.mecanum = 1  # Switch to pure mecanum mode
                if self.planner.isValidPath(self.path):
                    # If we arrived...
                    if self.pose_is_close(self.pose[:2], self.target):
                        # ok, say we arrived. what now????
                        self.drive.brake()
                        self.target = None
                        self.path = None

                        # Post-traversal operations:
                        self.mode = "manual"  # just for now, idk.

                    else:
                        # We probably should have some sort of monitor in here.
                        # Good thing this function is non-blocking (for the most part, path does block)
                        self.drive.cartesian(*self.path_traversal())

                else:
                    self.drive.brake()
                    self.path = self.planner.plan(self.pose[:2], list(self.target))
            else:  # we can just drive automatically, trying to discover new thingamajiggies :)
                self.drive.mecanum = 0  # Switch to differential drive
                self.lidar.self_nav()  # TODO: what parameters to use?


        def path_traversal(self) -> list:
            # This output gets unpacked into the drive.cartesian method
            # It must output direction, velocity and turn
            '''
            An idea for this path is just to turn at every point towards the next
            and drive in a straight line.
            # TODO: make sure it can drive in a straight line XD
            :return:
            '''
            raise NotImplementedError("IMPLEMENT THE DAMN PATH TRAVERSAL ALGO")

        def manual(self):
            pass

        def switch_mode(self):
            pass

        def main(self):
            pass

        def read_lidar(self):
            # TODO: if we use the non-threaded lidar for anything else, this may bring conflict.
            # An approach is to give the thread to slam, and update lidar.latest from that thread.
            if self.lt:
                return self.lidar.latest
            else:
                return self.lidar.read(self.rotate_reading)

        @staticmethod
        def pose_is_close(a, b, difference=1):  # remember to ensure equal units
            return ecd(np.array(a), np.array(b)) < difference

        def update_map(self):
            if self.lt:
                # TODO: test the computePoseChange thing.
                self.slam_pose = self.slam.update(*self.lidar.latest, self.kine.computePoseChange())
                sleep(1/8)  # about what it takes for the lidar to update.
                # TODO: implement a more rigorous approach
            else:  # expects this to be ran in a thread
                while not self.lt:
                    self.slam_pose = self.slam.update(*self.lidar.read(), self.kine.computePoseChange())

        def asf(self):

            # Lidar
            print("SLAM Test.")
            sleep(1)
            start = time()
            for i in range(10):
                self.drive.cartesian(0, 0, 1)
                distances, angles = self.read_lidar()
                print(self.slam.update(distances, angles))
            lidar_time = 1/(time()-start)
            print(f"FPS: {lidar_time}")
            print("Arm Test.")
            sleep(1)
            for i in range(10):
                #self.arm.moveTo((i+2, 90, 45))
                #sleep(0.5)
                # TODO: Arm is not working.
                pass

            print("Motor Test.")
            sleep(1)

            for i in range(628):
                x, y = np.cos(i/100), np.sin(i/100)
                self.drive.cartesian(x, y, 0.5)
                sleep(0.01)

        def exit(self, *args, **kwargs):
            self.drive.exit()
            self.lidar.exit()
            self.cam.stop()

    r = Robot()
    control = True

    if control:
        c = XboxController()
        def breaker():
            raise KeyError("Shutting down.")

        c.setTrigger("Back", breaker)
        c.setTrigger("Start", r.drive.switchDrive())
        while True:
            try:
                inputs = c.read()
                r.drive.drive(inputs[0], inputs[1], inputs[4], inputs[2])
            except KeyError:
                break
    else:
        r.asf()


