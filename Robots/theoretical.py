from simple_pid import PID
from extensions.tools import find_port_by_vid_pid, limit
import serial, struct, time, threading


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

        self.__battery_voltage = 0

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
        self.create_receive_threading()

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
            self.encoders = [struct.unpack('i', bytearray(ext_data[steps[i]:steps[i + 1]]))[0] for i in
                             range(len(steps) - 1)]
            timing = time.time()
            time_diff = timing - self.last_update
            self.last_update = timing
            for i in range(4):
                self.enc_speed[i] = (self.encoders[i] - self.prev_enc[i]) / time_diff

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
    def set_motor(self, speed_1=0, speed_2=0, speed_3=0, speed_4=0):
        try:
            t_speed_a = bytearray(struct.pack('b', self.__limit_motor_value(speed_1)))
            t_speed_b = bytearray(struct.pack('b', self.__limit_motor_value(speed_2)))
            t_speed_c = bytearray(struct.pack('b', self.__limit_motor_value(speed_3)))
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
            return self.__roll * 57.2957795, self.__pitch * 57.2957795, self.__yaw * 57.2957795
        else:
            return self.__roll, self.__pitch, self.__yaw

    # Get the battery voltage
    def get_battery_voltage(self):
        return self.__battery_voltage / 10.0

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


master = Rosmaster()


class Motor:
    def __init__(self, start_mode="position", index=0, sample_time=0.04):
        self.modes = {"position": [[0.365, 0, 0.045], (-100, 100), lambda: master.encoders[index]],
                      "speed": [[0.05, 0, 0.05], (-10, 10), lambda: master.enc_speed[index]]}
        self.current_mode = start_mode
        self.feeder = None  # gets assigned at self.switch_mode :)
        self.pid = PID()
        self.switch_mode(start_mode)
        self.sample_time = sample_time

        self.output = 0  # the motor output. pretty simple.
        self.stall = 0  # indicates that we're switching modes. The loop stops to give some room for all the changes.
        self.running = 1  # thread kill flag
        self.main_thread = threading.Thread(target=self.loop, daemon=True)
        self.main_thread.start()

    @property
    def target(self):
        return self.pid.setpoint

    @target.setter
    def target(self, value):
        self.pid.setpoint = value

    def switch_mode(self, mode):
        self.stall = 1  # stop the loop for a bit
        self.output = 0  # clear the motor output
        # apply the mode parameters
        self.pid.tunings, self.pid.output_limits, self.feeder = self.modes[mode]
        self.current_mode = mode

        # to make sure that the bot doesn't go haywire when we get into a new mode.
        if mode == "position":
            self.target = self.feeder()
        elif mode == "speed":
            self.target = 0

    def loop(self):
        while self.running:
            if self.stall:
                time.sleep(1)
                self.stall = 0
            reading = self.feeder()
            if self.current_mode == "position":
                self.output = self.pid(reading)
            elif self.current_mode == "speed":
                self.output += self.pid(reading)
            time.sleep(self.sample_time)

    def exit(self):
        self.running = 0
        self.main_thread.join()

    def __call__(self, value):
        self.target = value

    def __del__(self):
        self.exit()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.exit()
