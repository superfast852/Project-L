from Robots.RM_HAL import driver
from time import sleep
from simple_pid import PID
from matplotlib import pyplot as plt


def goTo(ticks, out=lambda x: driver.set_motor(0, x), inp=lambda: driver.encoders[1]):
    pid = PID(0.365, 0, 0.045, ticks, sample_time=0.04)
    pid.output_limits = (-100, 100)
    out(0)
    prev_read = 0
    settle_cnt = 0
    position = []
    while settle_cnt < 20 or abs(prev_read-ticks) > 10:
        reading = inp()
        position.append(reading)
        out(int(pid(reading)))
        if abs(reading - prev_read) < 10:
            if settle_cnt > 20:
                settle_cnt = 0
            else:
                settle_cnt += 1
        else:
            if settle_cnt >= 0:
                settle_cnt -= 1
        prev_read = reading
        sleep(0.04)
    return position


target = -10000
y = goTo(target)
print(y[-1])
x = range(len(y))
plt.plot(x, y)
plt.plot(x, [target]*len(x))
plt.show()
