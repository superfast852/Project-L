import numpy as np
from extensions.XboxController import XboxController


def cartesian(x, y, speed=1, turn=0, notches=False):
    theta = np.arctan2(y, -x)

    if notches:
        notches = np.linspace(0, 2 * np.pi, 8, endpoint=False)

        # Find the closest notch
        theta = min(notches, key=lambda notch: abs(theta - notch))

    sin = np.sin(theta + np.pi / 4)
    cos = np.cos(theta + np.pi / 4)
    lim = max(abs(sin), abs(cos))

    lf = speed * cos / lim + turn
    rf = speed * sin / lim - turn
    lb = speed * sin / lim + turn
    rb = speed * cos / lim - turn

    if (speed + abs(turn)) > 1:
        lf /= speed + turn
        rf /= speed + turn
        lb /= speed + turn
        rb /= speed + turn

    return lf, rf, lb, rb


def tank(V, omega):
    V_FL_BL = V - omega / 2
    V_FR_BR = V + omega / 2

    # Find the maximum absolute value among all speeds
    max_speed = max(abs(V_FL_BL), abs(V_FR_BR))

    # If the max speed is greater than 1, we need to scale down all speeds
    if max_speed > 1:
        V_FL_BL /= max_speed
        V_FR_BR /= max_speed
    return V_FL_BL, V_FR_BR, V_FL_BL, V_FR_BR


def get_directions(cmd):
    return [
        (cmd[0] + cmd[1] + cmd[2] + cmd[3]) / 4,
        (cmd[0] - cmd[1] - cmd[2] + cmd[3]) / 4,
        (cmd[0] - cmd[1] + cmd[2] - cmd[3]) / 4
    ]

def drive(x, y, power, turn):
    global mecanum
    if mecanum:
        return cartesian(-y, -x, power, turn)
    else:
        return tank(power, -x)


c = XboxController()
running = True
mecanum = True


def switchMode():
    global mecanum
    mecanum = not mecanum


def exitter():
    global running
    running = False


def run():
    c.setTrigger("Start", exitter)
    c.setTrigger("Back", switchMode)
    while running:
        inps = c.read()
        cmd = drive(inps[0], inps[1], inps[4], inps[2])
        print(get_directions(cmd), cmd)


if __name__ == "__main__":
    run()
