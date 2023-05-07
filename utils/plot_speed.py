from matplotlib import pyplot as plt
from extensions.tools import smoothSpeed, getCoordinates
from Robots.HAL import Drive

drive = Drive()


fig, ax = plt.subplots(1, 1)

x = [i for i in range(-35, 35)]
y = [smoothSpeed(i, 0, 1, 0.1, 5) for i in x]
ax.plot(x, y, label="smoothSpeed")
plt.show()
x = [i for i in range(0, 360)]
y = []
for i in x:
    posX, posY = getCoordinates(i)
    y.append(drive.cartesian(posX, posY))
y1 = [i[0] for i in y]
y2 = [i[1] for i in y]
y3 = [i[2] for i in y]
y4 = [i[3] for i in y]
plt.plot(x, y1, label="lf")
plt.title("lf")
plt.show()
plt.plot(x, y2, label="rf")
plt.title("rf")
plt.show()
plt.plot(x, y3, label="lb")
plt.title("lb")
plt.show()
plt.plot(x, y4, label="rb")
plt.title("rb")
plt.show()

x = [i/100 for i in range(-100, 101)]
y = [drive.cartesian(0, 1, 1, i) for i in x]
y1 = [i[0] for i in y]
y2 = [i[1] for i in y]
y3 = [i[2] for i in y]
y4 = [i[3] for i in y]
plt.plot(x, y1, label="lf")
plt.title("lf")
plt.show()
plt.plot(x, y2, label="rf")
plt.title("rf")
plt.show()
plt.plot(x, y3, label="lb")
plt.title("lb")
plt.show()
plt.plot(x, y4, label="rb")
plt.title("rb")
plt.show()
