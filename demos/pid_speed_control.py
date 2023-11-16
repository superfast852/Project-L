from Robots.RM_HAL import driver
from time import sleep
from simple_pid import PID
from matplotlib import pyplot as plt
try:
    print(driver.get_battery_voltage())
    target = 1000
    pid = PID(0.05, 0, 0.05, target, sample_time=0.2)
    pid.output_limits = (-10, 10)
    prev_read = 0
    settle_cnt = 0
    position = []
    driver.set_motor()
    speed = 0
    while settle_cnt < 20 or abs(prev_read-target) > 10:
        reading = driver.enc_speed[1]
        print(reading)
        if reading > 4000:
            continue
        position.append(reading)
        speed += pid(reading)
        driver.set_motor(0, round(speed))
        if abs(reading - prev_read) < 70:
            if settle_cnt > 20:
                settle_cnt = 0
            else:
                settle_cnt += 1
        else:
            if settle_cnt >= 0:
                settle_cnt -= 1
        prev_read = reading
        sleep(0.2)
    print(speed)
    plt.plot(range(len(position)), position)
    driver.set_motor()
    plt.show()
except KeyboardInterrupt:
    driver.set_motor()