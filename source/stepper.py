import RPi.GPIO as GPIO
import time

class Stepper:

    steps_per_revolution: int = 200
    low_endstop: float = -20
    high_endstop: float = 75 

    def __init__(self, step_pin: int, dir_pin: int, ms1_pin: int, ms2_pin: int, starting_angle: float = 0) -> None:
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.ms1 = ms1_pin
        self.ms2 = ms2_pin
        self.angle = starting_angle

        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.ms1, GPIO.OUT)
        GPIO.setup(self.ms2, GPIO.OUT)

        # 16 microstepping by default - maximum precission and quiet
        self.set_ustep(16)


    def set_ustep(self, ustep: int) -> None:
        if ustep not in [2,4,8,16]: 
            print('WARNING: unvalid microstepping passed:', ustep)
            return
        if ustep == 2:
            GPIO.output(self.ms1, 1)
            GPIO.output(self.ms2, 0)
        elif ustep == 4:
            GPIO.output(self.ms1, 0)
            GPIO.output(self.ms2, 1)
        elif ustep == 8:
            GPIO.output(self.ms1, 0)
            GPIO.output(self.ms2, 0)
        elif ustep == 16:
            GPIO.output(self.ms1, 1)
            GPIO.output(self.ms2, 2)
        self.ustep = ustep

    
    def set_angle(self, angle: float) -> None:
        if not Stepper.low_endstop < angle < Stepper.high_endstop:
            raise ValueError('Angle is outside the stepper endstops')
 
        steps = self.degrees_to_steps(abs(angle - self.angle))
        dir_val = 0 if angle > self.angle else 1
        self.move(dir_val, steps)


    def move(self, dir_val: int, steps: int) -> None:
        degree_sign = -1 if dir_val else 1
        degrees_per_step = 1.8 * degree_sign / self.ustep

        GPIO.output(self.dir_pin, dir_val)
        for i in range(steps):
            # 500 us square pulse
            GPIO.output(self.step_pin, 1)
            time.sleep(0.0005)
            GPIO.output(self.step_pin, 0)
            time.sleep(0.0005)
            self.angle += degrees_per_step

            # Delay to control speed
            delay = 0.005 - (0.004*(steps-i)/steps) 
            time.sleep(delay)


    def degrees_to_steps(self, degrees: float) -> int:
        steps = int((degrees * Stepper.steps_per_revolution * self.ustep)/360)
        return steps

    
if __name__ == '__main__':

    GPIO.setmode(GPIO.BCM)

    stepper1 = Stepper(23, 18, 25, 24)
    stepper2 = Stepper(27, 22, 4, 17)
    stepper3 = Stepper(5, 6, 19, 13)

    steppers = (stepper1, stepper2, stepper3)

    angle = 10
    while True:
        for stepper in steppers:
            stepper.set_angle(angle)
        angle *= -1


