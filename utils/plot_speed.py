from matplotlib import pyplot as plt
from extensions.tools import smoothSpeed


test_cases = [(-5, 5), (62.2, 510), (78, 77)]

for test in test_cases:
    y = smoothSpeed(test[0], test[1], steps=20)
    x = [i for i in range(len(y))]
    plt.plot(x, y)
    plt.show()
