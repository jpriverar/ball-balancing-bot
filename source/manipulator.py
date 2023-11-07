import numpy as np
from threading import Thread
import RPi.GPIO as GPIO
from stepper import Stepper
from inverse_kinematics import transformation_matrix_homogenous, compute_alpha, compute_theta
import time

class RRSManipulator:
    def __init__(self, base_radius: float, platform_radius: float, low_arm_length: float, high_arm_length: float) -> None:
        self.origin1 = np.array([[0,0,0]]).T
        self.h1 = base_radius
        self.h2 = platform_radius
        self.l1 = low_arm_length
        self.l2 = high_arm_length
        self.__init_joint_coordinates()

        stepper1 = Stepper(23, 18, 25, 24, 0)
        stepper2 = Stepper(5, 6, 19, 13, 0)
        stepper3 = Stepper(27, 22, 4, 17, 0)
        self.steppers = (stepper1, stepper2, stepper3)


    def home(self):
        self.set_motor_angles([0,0,0])

    
    def set_motor_angles(self, angles: list):
        if len(angles) != 3:
            print("ERROR: must pass 3 angles!")
            return

        threads = []
        for stepper, angle in zip(self.steppers, angles):
            th = Thread(target=self.set_motor_angle, args=(stepper, angle))
            threads.append(th)
            th.start()

        for th in threads:
            th.join()
            

    def set_motor_angle(self, stepper: Stepper, angle: float):
        stepper.set_angle(angle)


    def get_actuator_angles(self, offset, x_angle, y_angle):
        angles = []

        # Get the platform leg points seen from the base origin perspective
        platform_joints_pers = np.dot(transformation_matrix_homogenous(np.radians(x_angle), np.radians(y_angle), 0, offset), self.platform_joints)

        # Get revolute joint angles and passive revolute joint positions
        angle_shift = 0
        for i in range(3):
            # Points in the leg plane
            a_shifted = np.dot(transformation_matrix_homogenous(0, 0, np.radians(angle_shift), 0), self.base_joints[:,i])
            b_shifted = np.dot(transformation_matrix_homogenous(0, 0, np.radians(angle_shift), 0), platform_joints_pers[:,i])
            angle_shift += 120 # Each leg is 120 degrees apart

            # Required angle for the leg
            dist = b_shifted - a_shifted
            gamma = np.arctan2(dist[2], dist[1])
            alpha = compute_alpha(self.l1, self.l2, dist)
            theta = compute_theta(self.l1, self.l2, alpha, gamma)
            angles.append(np.degrees(theta))
        return angles

    
    def __init_joint_coordinates(self):
        # Base leg points
        self.base_joints = np.array([[0,                    self.h1,   0, 1], 
                                     [np.sqrt(3)*self.h1/2, -self.h1/2, 0, 1],
                                     [-np.sqrt(3)*self.h1/2, -self.h1/2, 0, 1]]).T
        
        # Platform leg points seen from the platform origin
        self.platform_joints = np.array([[0,                    self.h2, 0, 1],
                                         [np.sqrt(3)*self.h2/2, -self.h2/2, 0, 1], 
                                         [-np.sqrt(3)*self.h2/2, -self.h2/2, 0, 1]]).T



if __name__ == '__main__':
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    bot = RRSManipulator(base_radius = 6.0,
                   platform_radius = 8.0,
                   low_arm_length = 4.5,
                   high_arm_length = 9.0)
    
    #bot.home()
    #bot.set_motor_angle(bot.steppers[2], 20)


    x_angles = np.linspace(10, -10, 100)
    y_angles = np.linspace(10, -10, 100)
    offsets = np.linspace(8, 10, 100)
    #offsets = np.ones(100)*8
    #x_angles = [0,0,0,0,0]
    #y_angles = [0,0,0,0,0]
    #offsets = [8,8.5,9,9.5,10]
    

    for i in range(len(x_angles)):
        angles = bot.get_actuator_angles(offsets[i], x_angles[i], y_angles[i])
        print(offsets[i], x_angles[i], y_angles[i], "-->", angles)
        bot.set_motor_angles(angles)
