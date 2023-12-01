import numpy as np
from calibration_receiver import load_calibration_params
import time

class Controller:
    def __init__(self):
        params = load_calibration_params
        self.kp = params['controller']['kp']
        self.kd = params['controller']['kd']
        self.ki = params['controller']['kd']

        # Dervative and Integral errors
        self.err_prev = 0
        self.err_int = 0


    def get_output(self, target, actual):
        err_p = target - actual
        err_dev = err_p - self.err_prev
        self.err_int += err_p
        return err_p*self.kp + err_dev*self.kd + self.err_int*self.ki
        
