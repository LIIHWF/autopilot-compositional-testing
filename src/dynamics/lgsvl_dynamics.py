from .common import Dynamics
import math


class LgsvlDynamics(Dynamics):
    def __init__(self, vl = 80/3.6):
        self.vl = vl
    
    def acc_speed(self, v, t):
        vl = self.vl
        if t <= 4:
            return vl - (vl - v) * math.e ** (-3/10*t**2 - t)
        else:
            return vl - (vl - v) * math.e ** (24/5 - 17/5*t)
    
    def acc_distance(self, v, t):
        vl = self.vl
        if t <= 4:
            return -math.e ** (5/6) * math.sqrt(5*math.pi/6) * (vl - v) * math.erf((5 + 3 * t) / math.sqrt(30)) + vl * t + \
                math.e ** (5/6) * math.sqrt(5*math.pi/6) * (vl - v) * math.erf(5 / math.sqrt(30))
        else:
            return 5 / 17 * math.e ** (24/5 - 17/5*t) * (vl - v) + vl * t + \
                self.acc_distance(v, 4) - \
                    5 / 17 * math.e ** (24/5 - 17/5*4) * (vl - v) - 4 * vl
    
    @classmethod
    def B(cls, v):
        return v / 4.0
    
    def AT(self, v, x):
        vl = self.vl
        low_t = 0
        high_t = 100
        while high_t - low_t > 1e-6:
            mid_t = (low_t + high_t) / 2
            if self.acc_distance(v, mid_t) < x:
                low_t = mid_t
            else:
                high_t = mid_t
        return high_t
    
    def AV(self, v, x):
        vl = self.vl
        at = self.AT(v, x)
        return self.acc_speed(v, at)
