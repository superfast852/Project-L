"""

Path tracking simulation with pure pursuit steering and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)

"""
import numpy as np
import math
import matplotlib.pyplot as plt
from _pickle import load
from threading import Thread
import time

with open("valid_path.pkl", "rb") as f:
    path = load(f)[0]
    endpoints = np.append(path[:, 0], [path[-1, 1]], axis=0)
    cx = endpoints[:, 0].astype(float)
    cy = endpoints[:, 1].astype(float)
    cx /= 10.0
    cy /= 10.0
# Parameters
k = 0.1  # look forward gain
Lfc = 2.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick
WB = 2.9  # [m] wheel base of vehicle

show_animation = True


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class MecanumKinematics:  # units in centimeters.
    # lx, ly = 13.2, 8.5
    def __init__(self, radius=10, wheel2centerDistance=21.7):
        self.tpr = 362  # Output revolutions (rpm=333.333)
        self.r = radius
        self.revs = [0, 0, 0, 0]  # How many times every wheel has turned. Good luck with that.
        self.w2c = wheel2centerDistance
        # these take the amount of rotations per wheel and return the movement in centimeters.
        self.y = lambda lf, rf, lb, rb: (lf - rf - lb + rb) * (self.r / 4)
        self.x = lambda lf, rf, lb, rb: (lf + rf + lb + rb) * (self.r / 4)
        self.w = lambda lf, rf, lb, rb: (lf - rf + lb - rb) * (self.r / (4 * self.w2c))
        self.pose = [0, 0, 0]
        self.prev_ticks = [0, 0, 0, 0]
        self.timestampSecondsPrev = None
        self.thread = Thread(target=self.updatePosition, daemon=True)
        self.thread.start()

    def updatePosition(self):
        while True:
            self.revs = [i/self.tpr for i in driver.encoders]  # this turns the pose into revolution-based
            # This means that 1 turn on a wheel is 10cm of distance. Therefore,
            self.pose[0] = self.x(self.revs[0], self.revs[1], self.revs[2], self.revs[3])
            self.pose[1] = self.y(self.revs[0], self.revs[1], self.revs[2], self.revs[3])
            self.pose[2] = self.w(self.revs[0], self.revs[1], self.revs[2], self.revs[3])
            time.sleep(1 / 30)

    def getTicks(self, x, y, w):  # This already has the right hand rule coordinate system
        recip = 1 / self.r
        turn = self.w2c * w
        xy = x + y
        notxy = x - y
        return [recip * (notxy - turn), recip * (xy + turn), recip * (xy - turn), recip * (notxy + turn)]


class MecaState:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.kinematics = MecanumKinematics()  # MecanumKinematics instance

    def update(self, a, delta):
        # Assuming 'a' is linear acceleration and 'delta' is angular velocity

        # Update linear velocity
        self.v += a * dt

        wheel_rotations = self.cartesian(self.v, 0, 1, delta)  # Please note, this outputs values in [-1, 1].
        self.x, self.y, self.yaw = self.kinematics.pose

    def cartesian(self, x, y, speed=1, turn=0):
        rawTheta = math.atan2(y, x)
        theta = rawTheta - math.pi / 2 if rawTheta > math.pi / 2 else 2 * math.pi - rawTheta

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

        return lf, rf, lb, rb


class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)


def proportional_control(target, current):
    a = Kp * (target - current)

    return a


class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def main():
    #  target course
    #cx = np.arange(0, 50, 0.5)
    #cy = [(math.sin(ix / 5.0)**2) * (ix*ix) / 80.0 for ix in cx]

    target_speed = 10.0 / 3.6  # [m/s]

    T = 100.0  # max simulation time

    # initial state
    state = State(x=-0.0, y=-3.0, yaw=0.0, v=0.0)

    lastIndex = len(cx) - 1
    time = 0.0
    states = States()
    states.append(time, state)
    target_course = TargetCourse(cx, cy)
    target_ind, _ = target_course.search_target_index(state)

    while T >= time and lastIndex > target_ind:

        # Calc control input
        ai = proportional_control(target_speed, state.v)
        di, target_ind = pure_pursuit_steer_control(
            state, target_course, target_ind)

        state.update(ai, di)  # Control vehicle

        time += dt
        states.append(time, state)

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plot_arrow(state.x, state.y, state.yaw)
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(states.x, states.y, "-b", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

    # Test
    assert lastIndex >= target_ind, "Cannot goal"

    if show_animation:  # pragma: no cover
        plt.cla()
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(states.x, states.y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()