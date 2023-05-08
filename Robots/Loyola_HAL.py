from rplidar import RPLidar
import math
from breezyslam.sensors import RPLidarA1
from extensions.tools import getAngle, inTolerance, smoothSpeed
import time
from RPi import GPIO
sim = 0
GPIO.setmode(GPIO.BCM)

class Drive:  # TODO: Implement self.max properly
    def __init__(self, mag=None, max_speed=1):
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
            self.right_dir = lambda x: GPIO.output(19, x)
            self.right_speed = lambda x: self._r_pwm.ChangeDutyCycle(x)
            print("INIT Done.")

            from threading import Thread
            self.thread_life = 1
            self.thread = Thread(target=self.comms)
            self.thread.start()

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
        while self.thread_life:
            self.left_dir(self.lf > 0)
            self.left_speed(abs(self.lf*100))
            self.right_dir(self.rf > 0)
            self.right_speed(abs(self.rf*100))
            time.sleep(0.05)

    def drive(self, x, y, power, turn, flooring=False):
        if self.mecanum:
            return self.cartesian(x, y, power, turn, flooring)
        else:
            return self.tank(x, power, turn)

    def switchDrive(self):
        self.mecanum = not self.mecanum

    def brake(self):
        self.lf, self.rf, self.lb, self.rb = 0, 0, 0, 0

    def exit(self):
        self.mecanum = 2
        self.thread_life = False


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
    # TODO: Add angle rotation
    def __init__(self, rotation=0):
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
