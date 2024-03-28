import time
from threading import Thread
from extensions.tools import getAngle, XboxController
import math


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
        self.x = lambda lf, rf, lb, rb: (lf - rf - lb + rb) * (1 / 4)
        self.y = lambda lf, rf, lb, rb: (lf + rf + lb + rb) * (1 / 4)
        self.w = lambda lf, rf, lb, rb: (lf - rf + lb - rb) * (1 / 4)
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
        #print(f"Theta: {theta}, RawTheta: {rawTheta}")
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
            print(("Mecanum" if self.mecanum else "Tank") + f": {round(self.lf, 2)}, {round(self.rf, 2)}, {round(self.lb, 2)}, {round(self.rb, 2)}")
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

    def exit(self):
        self.brake()
        time.sleep(0.1)
        self.thread_life = 0



if __name__ == '__main__':
    drive = Drive()
    controller = XboxController()
    pulse_store = False
    while True:
        values = controller.read()
        mode, pulse_store = controller.edge(controller.Start, pulse_store)
        if mode:
            drive.switchDrive()
        speeds = drive.drive(values[0], values[1], values[4], values[2])
        #print(round(drive.x(*speeds), 4), round(drive.y(*speeds), 4), round(drive.w(*speeds), 4), getAngle(values[1], values[0]))
        time.sleep(0.1)