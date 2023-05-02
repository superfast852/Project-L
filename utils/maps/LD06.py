from numpy.random import randn
from breezyslam.sensors import Laser


class LD06(Laser):
    def __init__(self):
        from numpy.random import randn
        Laser.__init__(self, 360, 10, 360, 12000, 0, 0)
        self.map = [0] * 360

    def read(self):
        self._update()
        return list(self.map)

    def _update(self):
        self.map = randn(360)

    def exit(self):
        self.map = None
