import math
import time
import serial
from threading import Thread


def getAngle(x, y):
    # Calculate the angle in radians and rotate it counterclockwise by 90 degrees
    if x == 0 and y == 0:
        return 0.0
    angle = math.degrees(math.atan2(y, x) - math.pi / 2)
    if angle < 0:
        angle += 360
    # Return the angle in degrees
    return int(angle)


def smoothSpeed(current, target, speed_lim=1, min_speed=0.1, smoothing_spread=10):
    distance = current-target
    try:
        direction = -distance/abs(distance)
        speed = min(9, (distance/10)**2/smoothing_spread)
        output = (1+speed)*direction/10*speed_lim
        return output if abs(output) > min_speed else min_speed*direction
    except ZeroDivisionError:
        return 0


class Drive:
    def __init__(self, com='/dev/ttyACM0', baud=115200, max_speed=1, sim=False):
        self.lf = 0
        self.rf = 0
        self.lb = 0
        self.rb = 0
        self.max = max_speed
        self.mecanum = 1
        self.distance_per_tick = 0.1427995  # in mm
        self.board = serial.Serial(com, baud) if not sim else NotImplemented
        self.comm_thread = Thread(target=self.comms)
        self.comm_thread.start()
        self.encoders = [0] * 4
        self.current_pos = (0, 0, 0)

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

    def tank(self, x, power, turn=False):
        if power != 0:
            lf = lb = round((power + ((power / abs(power)) * min(0, x))), 3)
            rf = rb = round((power - ((power / abs(power)) * max(0, x))), 3)
        elif turn:
            lf = lb = power*turn
            rf = rb = -lf
        else:
            lf, rf, lb, rb = 0, 0, 0, 0
        self.lf = lf
        self.rf = rf
        self.lb = lb
        self.rb = rb
        return lf, rf, lb, rb

    def moveTo(self, x, y, speed):
        while x != self.current_pos[0] and y != self.current_pos[1]:
            self.cartesian(smoothSpeed(self.current_pos[0], x, 1))

    def comms(self):
        while True:
            self.board.write(bytearray([self.lf, self.rf, self.lb, self.rb]))  # TODO: Send as Strings. This will crash.
            time.sleep(0.1)
            self.encoders = self.board.read(4)
            self.calc_odometry()

    def calc_odometry(self):
        # self.current_pos[2] = self.initial_angle - magnetometer.read()
        pass

    def drive(self, x, y, power, turn):
        if self.mecanum == 1:
            return drive.cartesian(x, y, power, turn)
        else:
            return drive.tank(x, power, turn)

    def switchDrive(self):
        self.mecanum = not self.mecanum


if __name__ == "__main__":
    from controller import XboxController
    drive = Drive(sim=True)
    joy = XboxController(0.15)
    states = {"RB": 0}
    while True:
        reads = joy.read()
        edge, states["RB"] = joy.edge(joy.RB, states['RB'])
        drive.switchDrive() if edge else print(drive.drive(reads[0], reads[1], reads[4], reads[2]))

