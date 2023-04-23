from matplotlib import pyplot as plt
from utils import smoothSpeed
from interface import Drive

drive = Drive()


fig, ax = plt.subplots(1, 1)

x = [i for i in range(-100, 0)]
y = [smoothSpeed(i, 0, 1, 0.1, 5) for i in x]
ax.plot(x, y, label="smoothSpeed")
plt.show()
x = [i for i in range(0, 360)]
y = [drive.angle_cartesian(i) for i in x]
y1 = [i[0] for i in y]
y2 = [i[1] for i in y]
y3 = [i[2] for i in y]
y4 = [i[3] for i in y]
plt.plot(x, y1, label="lf")
plt.show()
plt.plot(x, y2, label="rf")
plt.show()
plt.plot(x, y3, label="lb")
plt.show()
plt.plot(x, y4, label="rb")
plt.show()
