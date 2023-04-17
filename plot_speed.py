from matplotlib import pyplot as plt
from utils import smoothSpeed

fig, ax = plt.subplots(1, 1)

x = [i for i in range(-100, 0)]
y = [smoothSpeed(i, 0, 1, 0.1, 5) for i in x]

ax.plot(x, y)
plt.show()
