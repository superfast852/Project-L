# TODO: Add logging, adopt new coordinate system (Right Hand Rule: X=Forward, Y=Left, Z=Up)
# NOTE: In the driver code, motors 1 and 3 are flipped.

from threading import Thread
from extensions.tools import getAngle, smoothSpeed, math, find_port_by_vid_pid, np, limit
from breezyslam.sensors import RPLidarA1
from rplidar import RPLidar, RPLidarException
from itertools import groupby
from operator import itemgetter
import struct
import time
import serial
import threading
import ikpy.chain
global_start = time.time()


# V1.7.3
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
        self.last_update = time.time()

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
        time.sleep(.002)

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
            timing = time.time()
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
        time.sleep(0.002)

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
                task_receive = threading.Thread(target=self.__receive_data, name=name1)
                task_receive.setDaemon(True)
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
            time.sleep(self.__delay_time)
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
            time.sleep(self.__delay_time)
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
            time.sleep(self.__delay_time)
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
            time.sleep(self.__delay_time)
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
            time.sleep(self.__delay_time)
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
            time.sleep(self.__delay_time)
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
            time.sleep(self.__delay_time)
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
            time.sleep(self.__delay_time)
            time.sleep(.1)
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
            time.sleep(.1)
        else:
            print("set_car_type input invalid")


driver = Rosmaster(car_type=Rosmaster.CARTYPE_X3)


# This is a raw driving output class.
# It's only function is to move the robot in the direction and speed directed.
# Custom driving functions can build on top of this class.
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
        self.comm_thread = Thread(target=self.comms)
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
            time.sleep(1/update_freq)

    def drive(self, x, y, power, turn):
        if self.mecanum:
            return self.cartesian(x, y, power, turn)
        else:
            return self.tank(power, turn)

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

    def exit(self):
        self.brake()
        time.sleep(0.1)
        self.thread_life = 0

    def get_directions(self):
        return [
            (self.lf - self.rf - self.lb + self.rb)/4,
            (self.lf + self.rf + self.lb + self.rb)/4,
            (self.lf - self.rf + self.lb - self.rb)/4
        ]



class MPU:
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
        from adafruit_servokit import ServoKit
        try:
            self.chain = ikpy.chain.Chain.from_urdf_file('../Resources/arm.urdf',
                                                     active_links_mask=[bool(i%2) for i in range(10)])
        except FileNotFoundError:
            self.chain = ikpy.chain.Chain.from_urdf_file('./Resources/arm.urdf',
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

    def moveTo(self, target, vector=False):
        if vector:
            target = self.v2p(target)
        pose = self.chain2pose(self.chain.inverse_kinematics(target, initial_position=self.pose2chain(self.pose)))
        for i, angle in enumerate(pose[:5]):
            self.arm[i] = angle

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
        from _thread import interrupt_main
        from time import sleep
        while True:
            voltage = self.get_voltage()
            if voltage <= self.threshold:
                print("[CRITICAL] Battery levels too low! Voltage: ", voltage)
                for i in range(5):
                    driver.set_beep(100)
                    sleep(0.2)
                interrupt_main()
                break
            time.sleep(self.polling)


class RP_A1(RPLidarA1):
    VID = 0x10c4
    PID = 0xea60
    def __init__(self, com="/dev/ttyUSB2", baudrate=115200, timeout=3, rotation=0):
        super().__init__()
        try:
            port = find_port_by_vid_pid(self.VID, self.PID)
            self.lidar = RPLidar(port, baudrate, timeout)
        except RPLidarException:
            self.lidar = RPLidar(com, baudrate, timeout)
        print(self.lidar.get_info(), self.lidar.get_health())
        self.scanner = self.lidar.iter_scans(max_buf_meas=8192)
        self.lidar.clean_input()
        next(self.scanner)
        self.rotation = rotation % 360
        # last_scan = self.read()  # Return this for when we must clear the buffer if we don't read fast enough.

    def read(self):  # It's totally possible to move this to a thread.
        items = [item for item in next(self.scanner)]
        angles, distances = list(zip(*items))[1:]
        if self.rotation != 0:
            distances, angles = list(zip(*self.rotate_lidar_readings(zip(distances, angles))))
        return list(distances), list(angles)

    def exit(self):
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()

    def autoStopCollision(self, collision_threshold):
        # This returns only the clear angles.
        return [int(angle) for (distance, angle) in zip(self.read()) if distance <= collision_threshold]

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
        self.x = lambda lf, rf, lb, rb: (lf - rf - lb + rb) * (self.r / 4)
        self.y = lambda lf, rf, lb, rb: (lf + rf + lb + rb) * (self.r / 4)
        self.w = lambda lf, rf, lb, rb: (lf - rf + lb - rb) * (self.r / (4 * self.w2c))
        self.vec = (0, 0)
        self.pose = [0, 0, 0]
        self.prev_ticks = [0, 0, 0, 0]
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
            time.sleep(1 / 30)

    def getTicks(self, x, y, w):  # This already has the right hand rule coordinate system
        recip = 1 / self.r
        turn = self.w2c * w
        xy = x + y
        notxy = x - y
        return [recip * (notxy - turn), recip * (xy + turn), recip * (xy - turn), recip * (notxy + turn)]

    def computePoseChange(self, timestamp):
        timestamp = time.time() - global_start if timestamp is None else timestamp
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