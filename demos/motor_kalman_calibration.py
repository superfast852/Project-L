from matplotlib import pyplot as plt
from Robots.RM_HAL import driver
import numpy as np
from time import sleep


fig, ax = plt.subplots(nrows=2, ncols=2)
f = lambda x: round(np.sin((x/5000)*np.pi)*100)   # x//50 - 100  #(80*(x/1000))+20

timestamps = list(range(10000))
sp = []
ticks = []
driver._setm()

for i in timestamps:
    out = [f(i)]*4
    driver._setm(*out)
    ticks.append(driver.enc_speed.copy())
    sp.append(out)
    if (i%10 == 0):
        print(f"{i/10}% Done.")

driver._setm()
ticks = np.array(ticks).T
sp = (np.array(sp).T)*20
print(ticks)

ax[0][0].plot(timestamps, ticks[0])
ax[0][0].plot(timestamps, sp[0])
ax[0][1].plot(timestamps, ticks[1])
ax[0][1].plot(timestamps, sp[1])
ax[1][0].plot(timestamps, ticks[2])
ax[1][0].plot(timestamps, sp[2])
ax[1][1].plot(timestamps, ticks[3])
ax[1][1].plot(timestamps, sp[3])
plt.show()