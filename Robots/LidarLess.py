from .HAL import Drive, io as GPIO, time
from networktables import NetworkTables


class LDrive(Drive):  # Override Communications
    def comms(self, com, baud, update_freq=10):
        GPIO.setup(20, GPIO.OUT)
        self._lf_pwm = GPIO.PWM(20, 1000)
        self._lf_pwm.start(0)

        GPIO.setup(13, GPIO.OUT)
        self._rf_pwm = GPIO.PWM(13, 1000)
        self._rf_pwm.start(0)

        GPIO.setup(16, GPIO.OUT)
        self._lb_pwm = GPIO.PWM(16, 1000)
        self._lb_pwm.start(0)

        GPIO.setup(7, GPIO.OUT)
        self._rb_pwm = GPIO.PWM(7, 1000)
        self._rb_pwm.start(0)

        GPIO.setup(12, GPIO.OUT)
        GPIO.setup(6, GPIO.OUT)
        GPIO.setup(21, GPIO.OUT)
        GPIO.setup(8, GPIO.OUT)
        GPIO.output(12, 0)
        GPIO.output(6, 0)
        GPIO.output(21, 0)
        GPIO.output(8, 0)

        self.lfDir      = lambda x: GPIO.output(21, x)
        self.lfSpeed    = lambda x: self._lf_pwm.ChangeDutyCycle(x)
        self.rfDir      = lambda x: GPIO.output(6, x)
        self.rfSpeed    = lambda x: self._rf_pwm.ChangeDutyCycle(x)
        self.lbDir      = lambda x: GPIO.output(12, x)
        self.lbSpeed    = lambda x: self._lb_pwm.ChangeDutyCycle(x)
        self.rbDir      = lambda x: GPIO.output(8, x)
        self.rbSpeed    = lambda x: self._rb_pwm.ChangeDutyCycle(x)

        while self.thread_life:
            self.lfDir(self.lf > 0)
            self.lfSpeed(abs(self.lf * 100))
            self.rfDir(self.rf > 0)
            self.rfSpeed(abs(self.rf * 100))
            self.lbDir(self.lb > 0)
            self.lbSpeed(abs(self.lb * 100))
            self.rbDir(self.rb > 0)
            self.rbSpeed(abs(self.rb * 100))
            time.sleep(1 / update_freq)


class Robot:
    def __init__(self):
        NetworkTables.initialize()
        self.drive = LDrive()
        self.io = GPIO

    def exit(self):
        self.drive.exit()
        self.io.cleanup()
