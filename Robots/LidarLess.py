from .HAL import Drive, io as GPIO, time
from networktables import NetworkTables


class LDrive(Drive):  # Override Communications

    def comms(self, com, baud, update_freq=10):
        GPIO.setup(20, GPIO.OUT)
        lf_pwm = GPIO.PWM(20, 1000)
        lf_pwm.start(0)

        GPIO.setup(13, GPIO.OUT)
        rf_pwm = GPIO.PWM(13, 1000)
        rf_pwm.start(0)

        GPIO.setup(16, GPIO.OUT)
        lb_pwm = GPIO.PWM(16, 1000)
        lb_pwm.start(0)

        GPIO.setup(7, GPIO.OUT)
        rb_pwm = GPIO.PWM(7, 1000)
        rb_pwm.start(0)

        GPIO.setup(12, GPIO.OUT)
        GPIO.setup(6, GPIO.OUT)
        GPIO.setup(21, GPIO.OUT)
        GPIO.setup(8, GPIO.OUT)
        GPIO.output(12, 0)
        GPIO.output(6, 0)
        GPIO.output(21, 0)
        GPIO.output(8, 0)

        while self.thread_life:
            GPIO.output(21, self.lf > 0)
            lf_pwm.ChangeDutyCycle(abs(self.lf * 100))
            GPIO.output(6, self.rf > 0)
            rf_pwm.ChangeDutyCycle(abs(self.rf * 100))
            GPIO.output(12, self.lb > 0)
            lb_pwm.ChangeDutyCycle(abs(self.lb * 100))
            GPIO.output(8, self.rb > 0)
            rb_pwm.ChangeDutyCycle(abs(self.rb * 100))
            time.sleep(1 / update_freq)


class Robot:
    def __init__(self):
        NetworkTables.initialize()
        self.drive = LDrive()
        self.io = GPIO

    def exit(self):
        self.drive.exit()
        self.io.cleanup()
