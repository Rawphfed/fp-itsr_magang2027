#!/usr/bin/env python3
import math


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    return max(-limit, min(limit, value))


class PID:
    def __init__(
        self,
        kp,
        ki=0.0,
        kd=0.0,
        o_lim=math.inf,
        i_lim=math.inf,
        yaw=False,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.o_lim = abs(o_lim)
        self.i_lim = abs(i_lim)
        self.yaw = yaw
        self.reset()

    def reset(self):
        self.i_term = 0.0
        self.prev_measurement = None

    ## Error calculation
    def _diff(self, a, b):
        diff = a - b
        return wrap_angle(diff) if self.wrap else diff

    ## Update value of the PID of an element
    def update(self, setpoint, measurement, dt):
        error = self._diff(setpoint, measurement)
        valid_dt = dt > 0.0
        
        # Proportional part
        p_term = self.kp * error
        
        # Derivative part
        d_term = 0.0
        if valid_dt and self.prev_measurement is not None:
            rate = self._diff(measurement, self.prev_measurement) / dt
            d_term = -self.kd * rate
        self.prev_measurement = measurement

        # Integral part
        new_i = self.i_term
        if valid_dt:
            new_i = clamp(self.i_term + self.ki * error * dt, self.i_lim)

        # Anti windup (from what i understand, prevent I to accum extreme values when output is min or max)
        raw = p_term + new_i + d_term

        pushing_into_saturation = abs(raw) > self.o_lim and error * raw > 0.0
        if not pushing_into_saturation:
            self.i_term = new_i

        output = p_term + self.i_term + d_term
        return clamp(output, self.o_lim)
