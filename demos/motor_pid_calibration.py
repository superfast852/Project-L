from matplotlib import pyplot as plt
from Robots.RM_HAL import driver
import numpy as np
from time import sleep

fig, ax = plt.subplots(nrows=2, ncols=2)
f = lambda x: int(np.sin(x/500*np.pi)*100) # x: x//5 - 100  #(80*(x/1000))+20
print(f(0)*driver.mts/100)
timestamps = list(range(2000))
sp = []
ticks = []
og = []
#driver._setm(0, 0, 0, 100)


def avg(data):
    return sum(data)/len(data)


for i in timestamps:
    out = [f(i)]*4
    driver.set_motor(*out)
    #driver._setm(*out)
    #sleep(1/120)
    ticks.append(driver.enc_speed.copy())
    sp.append(out)
    og.append(driver.raw.copy()[3])
    if (i%10 == 0):
        print(f"{i/10}% Done.")

driver.set_motor()
driver._setm()
ticks = np.array(ticks).T
sp = (np.array(sp).T)*driver.mts/100
print(np.max(ticks, axis=1))
ax[0][0].plot(timestamps, ticks[0])
ax[0][0].plot(timestamps, sp[0])
ax[0][1].plot(timestamps, ticks[1])
ax[0][1].plot(timestamps, sp[1])
ax[1][0].plot(timestamps, ticks[2])
ax[1][0].plot(timestamps, sp[2])
ax[1][1].plot(timestamps, ticks[3])
ax[1][1].plot(timestamps, sp[3])

plt.show()

plt.plot(timestamps, og)
plt.show()