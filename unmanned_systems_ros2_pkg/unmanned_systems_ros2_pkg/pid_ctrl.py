#!/usr/bin/env python3

# this is the stuff that he had us do in lab 

class PID():
    def __init__(self, kp, ki, kd, dt) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.error = [0,0]

    def compute_error(self, des, actual):
        self.error[0] = des-actual 
        return self.error[0]
        
    def get_gains(self, des, actual):
        #p gains
        self.compute_error(des, actual)
        p = self.kp * self.error[0] 
        d = self.kd * (self.error[0] - self.error[1])/self.dt
        #update error
        self.error[1] = self.error[0]
        
        return p+d
        