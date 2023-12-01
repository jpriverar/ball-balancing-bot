
class Controller:
    def __init__(self, params):
        self.kp = params['kp']
        self.kd = params['kd']
        self.ki = params['kd']

        # Dervative and Integral errors
        self.err_prev = 0
        self.err_int = 0


    def get_output(self, target, actual):
        err_p = target - actual
        err_dev = err_p - self.err_prev
        self.err_int += err_p
        return err_p*self.kp + err_dev*self.kd + self.err_int*self.ki
        
