import numpy as np
from threading import Thread
from stepper import Stepper
import inverse_kinematics as ik

class RRSManipulator:
    def __init__(self, base_radius: float, platform_radius: float, low_arm_length: float, high_arm_length: float) -> None:
        self.origin1 = np.array([[0,0,0]]).T
        self.h1 = base_radius
        self.h2 = platform_radius
        self.l1 = low_arm_length
        self.l2 = high_arm_length

        stepper1 = Stepper(23, 18, 25, 24, -20)
        stepper2 = Stepper(27, 22, 4, 17, -20)
        stepper3 = Stepper(5, 6, 19, 13, -20)
        self.steppers = (stepper1, stepper2, stepper3)

    def home(self):
        threads = []
        for stepper in self.steppers:
            th = Thread(target=stepper.set_angle(0))
            threads.append(th)
            th.start()

        for th in threads:
            th.join()


if __name__ == '__main__':
    
    bot = RRSManipulator(base_radius = 6.0,
                   platform_radius = 8.0,
                   low_arm_length = 4.5,
                   high_arm_length = 9.0)
    
    bot.home()