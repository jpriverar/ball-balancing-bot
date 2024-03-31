import numpy as np

class Controller:
    def __init__(self, params):
        self.kp = params['kp']
        self.kd = params['kd']
        self.ki = params['kd']
        self.target = params['initial_target']
        self.norm_factor = params['norm_factor']

        # Dervative and Integral errors
        self.error_prev = 0
        self.error_int = 0


    def get_output(self, error):
        #err_p = self.target - error/self.norm_factor
        error_dev = error - self.error_prev
        self.error_int += error
        self.error_prev = error
        output = error*self.kp + error_dev*self.kd + self.error_int*self.ki
        return np.clip(output, -15, 15)