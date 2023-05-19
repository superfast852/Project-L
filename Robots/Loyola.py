from networktables import NetworkTables
from Robots.HAL import Drive, RP_A1, io as GPIO, time


class LDrive(Drive):  # Override Communications
    def comms(self, com, baud, update_freq=10):
        GPIO.setup(19, GPIO.OUT)
        self._l_pwm = GPIO.PWM(19, 1000)
        self._l_pwm.start(0)
        GPIO.setup(6, GPIO.OUT)
        self._r_pwm = GPIO.PWM(6, 1000)
        self._r_pwm.start(0)
        print("PWM Started.")
        GPIO.setup(26, GPIO.OUT)
        GPIO.setup(13, GPIO.OUT)
        GPIO.output(26, 0)
        GPIO.output(13, 0)

        self.left_dir = lambda x: GPIO.output(26, x)
        self.left_speed = lambda x: self._l_pwm.ChangeDutyCycle(x)
        self.right_dir = lambda x: GPIO.output(13, x)
        self.right_speed = lambda x: self._r_pwm.ChangeDutyCycle(x)
        print("INIT Done.")

        while self.thread_life:
            self.left_dir(self.lf > 0)
            self.left_speed(abs(self.lf * 100))
            self.right_dir(self.rf > 0)
            self.right_speed(abs(self.rf * 100))
            time.sleep(1/update_freq)


class Robot:

    def __init__(self):
        NetworkTables.initialize()
        self.lidar = RP_A1()  # LD06()
        self.drive = LDrive(collision_fn=self.lidar.autoStopCollision, arg=10)
        self.io = GPIO

    def exit(self):
        NetworkTables.stopServer()
        self.drive.exit()
        self.io.cleanup()
        self.lidar.exit()
        print("Robot exited.")