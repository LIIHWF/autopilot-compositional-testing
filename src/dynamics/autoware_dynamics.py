from .common import Dynamics

import math


def calc_judge_line_dist_with_jerk_limit(velocity, acceleration, max_stop_acceleration, max_stop_jerk, delay_response_time):
    if velocity <= 0.0:
        return 0.0

    # t0: subscribe traffic light state and decide to stop
    # t1: braking start (with jerk limitation)
    # t2: reach max stop acceleration
    # t3: stop

    t1 = delay_response_time
    x1 = velocity * t1

    v2 = velocity + (math.pow(max_stop_acceleration, 2) - math.pow(acceleration, 2)) / (2.0 * max_stop_jerk)

    if v2 <= 0.0:
        t2 = -1.0 * (max_stop_acceleration + math.sqrt(acceleration ** 2 - 2.0 * max_stop_jerk * velocity)) / max_stop_jerk
        x2 = velocity * t2 + acceleration * math.pow(t2, 2) / 2.0 + max_stop_jerk * math.pow(t2, 3) / 6.0
        return max(0.0, x1 + x2)

    t2 = (max_stop_acceleration - acceleration) / max_stop_jerk
    x2 = velocity * t2 + acceleration * math.pow(t2, 2) / 2.0 + max_stop_jerk * math.pow(t2, 3) / 6.0

    x3 = -1.0 * math.pow(v2, 2) / (2.0 * max_stop_acceleration)
    return max(0.0, x1 + x2 + x3)


class AutowareDynamics(Dynamics):
    def __init__(self, vl = 80/3.6):
        self.vl = vl

    def B(self, v):
        assert v >= 0
        return calc_judge_line_dist_with_jerk_limit(v, 0, -5, -5, 0)
    
    def AXT(self, v, w):
        assert w >= v
        if w - v < 1.5:
            # case 1
            a_max = math.sqrt(5 / 3 * (w - v))
            t1 = a_max
            v1 = v + t1 ** 2  / 2
            x1 = v * t1 + 1 / 3 * t1 ** 3
            t2 = a_max / 5
            x2 = v1 * t2 + 0.5 * t2 ** 2 - 5 / 6 * t2 ** 3
            return x1 + x2, t1 + t2
        else:
            # case 2
            t1 = 1
            v1 = v + 0.5 * t1 ** 2
            x1 = v * t1 + 1 / 6 * t1 ** 3
            v2 = w - 0.1
            t2 = (v2 - v1) / 1
            x2 = v1 * t2 + 0.5 * t2 ** 2
            t3 = 1 / 5
            x3 = v2 * t3 + 0.5 * t3 ** 2 - 5 / 6 * t3 ** 3
            return x1 + x2 + x3, t1 + t2 + t3

    def AV(self, v, x):
        vl = self.vl
        w_low = v
        w_high = vl
        while w_high - w_low > 1e-6:
            w = (w_low + w_high) / 2
            x_, t = self.AXT(v, w)
            if x_ < x:
                w_low = w
            else:
                w_high = w
        return w_low

    def AT(self, v, x):
        vl = self.vl
        w = self.AV(v, x)
        x_, t = self.AXT(v, w)
        return t + (x - x_) / vl
    
    def brake_trace(self, initial_speed):
        max_jerk = 5
        max_b = 5
        v = initial_speed
        xs = []
        vs = []
        b = 0
        dt = 0.001
        x = 0
        while v > 0:
            b = min(b + max_jerk * dt, max_b)
            v -= b * dt
            if v < 0:
                v = 0
            x += v * dt
            # x += v * dt - 0.5 * b * dt * dt
            xs.append(x)
            vs.append(v)
        return xs, vs
    
    def accelerate_trace_by_t(self, initial_speed, total_time):
        max_jerk = 1
        min_jerk = -5
        max_a = 1
        t = 0
        v = initial_speed
        a = 0
        xs = []
        vs = []
        dt = 0.01
        state = 0
        x = 0
        while t < total_time:
            if state == 1 or ( state == 0 and total_time - t < abs(a / min_jerk)):
                state = 1
                a = max(a + min_jerk * dt, 0)
            else:
                a = min(a + max_jerk * dt, max_a)
            v = min(v + a * dt, self.vl)
            x += v * dt
            t += dt
            xs.append(x)
            vs.append(v)
        return xs, vs

    def acceleration_trace(self, initial_speed, distance):
        t = self.AT(initial_speed, distance)
        return self.accelerate_trace_by_t(initial_speed, t)

    def fast_distance_by_top(self, initial_speed, distance):
        acc_v = self.AV(initial_speed, distance)
        dis_b = self.B(acc_v)
        return dis_b + distance

    def fast_trace(self, initial_speed, distance):
        # first accelerate then decelerate to 0, maximum speed is self.vl, the total distance is distance

        max_acc_distance = self.AXT(initial_speed, self.vl)[0]
        max_brake_distance = self.B(self.vl)
        if max_acc_distance + max_brake_distance < distance:
            # reach top, keep it then brake
            acc_trace = self.acceleration_trace(initial_speed, max_acc_distance)
            constant_trace = [acc_trace[-1], distance - max_brake_distance], [self.vl, self.vl]
            brake_trace = self.brake_trace(self.vl)
            for i in range(len(brake_trace[0])):
                brake_trace[0][i] += constant_trace[0][-1]
            return acc_trace[0] + constant_trace[0] + brake_trace[0], acc_trace[1] + constant_trace[1] + brake_trace[1]
        
        # using binary search for the top speed
        low = 0
        high = distance
        while high - low > 1e-2:
            mid = (low + high) / 2
            if self.fast_distance_by_top(initial_speed, mid) < distance:
                low = mid
            else:
                high = mid
        mid_distance = low
        
        # first accelerate to the top speed
        acc_trace = self.acceleration_trace(initial_speed, mid_distance)
        brake_trace = self.brake_trace(acc_trace[1][-1])
        for i in range(len(brake_trace[0])):
            brake_trace[0][i] += acc_trace[0][-1]
        return acc_trace[0] + brake_trace[0], acc_trace[1] + brake_trace[1]
