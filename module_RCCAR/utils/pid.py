# utils/pid.py
class PIDController:
    def __init__(self, kp, ki, tc, integral_max):
        self.kp = kp
        self.ki = ki
        self.tc = tc
        self.integral_max = integral_max
        self.integral = 0

    def calculate(self, error):
        self.integral += error * self.tc
        self.integral = max(min(self.integral, self.integral_max), -self.integral_max)
        control = self.kp * error + self.ki * self.integral
        return control