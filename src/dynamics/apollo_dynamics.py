import math
from .common import Dynamics

class ApolloDynamics(Dynamics):
    def __init__(self, vl = 80/3.6):
        self.vl = vl
        
    def B(self, v):
        assert v >= 0
        if v <= 27 / 2:
            return 2 * math.sqrt(6) * v ** (1.5) / 9
        else:
            return v ** 2 / 12 + 3 * v / 4 + 27 / 16
        
    def acc_distance(self, v):
        low_x = 0
        high_x = 100
        while high_x - low_x > 1e-6:
            mid_x = (high_x + low_x) / 2
            if self.acc_v(v, mid_x) < self.vl:
                low_x = mid_x
            else:
                high_x = mid_x
        return (low_x + high_x) / 2

    def acc_v(self, v, x):
        if x == 0:
            return v
        if v < (2 * x - 2) / 3:
            return -0.5 + math.sqrt(v ** 2 - 2 * v + 4 * x)
        return - 0.5 * v + (3 * v ** 2) / (4 * (v ** 3 + 4 * x ** 2 + 2 * x * math.sqrt(2 * v ** 3 + 4 * v ** 2)) ** (1 / 3)) + 3 / 4 * \
                                                (v ** 3 + 4 * x ** 2 + 2 * x * math.sqrt(2 * v ** 3 + 4 * v ** 2)) ** (1 / 3)

    def acc_t(self, v, x):
        av = self.AV(v, x)
        if 8 / 3 * (av - v) <= 4:
            return math.sqrt(6 * (av - v)) / 2
        else:
            return (av - v) / 2 + 3 / 4

    def AV(self, v, x):
        assert v >= 0 and x >= 0
        acc_dis = self.acc_distance(v)
        if x <= acc_dis:
            return self.acc_v(v, x)
        else:
            return self.vl
        
    def AT(self, v, x):
        assert v >= 0 and x >= 0
        acc_dis = self.acc_distance(v)
        if x <= acc_dis:
            return self.acc_t(v, x)
        else:
            return self.acc_t(v, acc_dis) + (x - acc_dis) / self.vl
