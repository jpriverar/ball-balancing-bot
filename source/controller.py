import numpy as np

class Controller:
    def __init__(self, params):
        self.kp = params['kp']
        self.kd = params['kd']
        self.ki = params['kd']
        self.target = params['initial_target']

        # Dervative and Integral errors
        self.err_prev = 0
        self.err_int = 0


    def get_output(self, error):
        err_p = self.target - error
        err_dev = err_p - self.err_prev
        self.err_int += err_p
        self.err_prev = err_p
        output = err_p*self.kp + err_dev*self.kd + self.err_int*self.ki
        return np.clip(output, -12, 12)
        
