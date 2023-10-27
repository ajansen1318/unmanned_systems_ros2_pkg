#!/usr/bin/env python3
import numpy as np


class PIDControl:
    def __init__(self, Kp, Ki, Kd) -> None:
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.error = [0, 0]

    def calculate_pid(self, desired, actual, dt):
        I_val = 0
        self.error[0] = desired - actual

        P_val = self.Kp * self.error[0]
        I_val += self.error[0] * dt
        D_val = self.Kd * (self.error[0] - self.error[1]) / dt

        PID_val = P_val + self.Ki * I_val + D_val
        self.error[1] = self.error[0]

        return PID_val, self.error
