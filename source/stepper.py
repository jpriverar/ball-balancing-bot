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

        # 16 microstepping
        GPIO.output(self.ms1, 1)
        GPIO.output(self.ms2, 1)
        self.ustep = 16

    
    def set_angle(self, angle: float) -> None:
        if not Stepper.low_endstop < angle < Stepper.high_endstop:
            return
 
        steps = self.degrees_to_steps(angle - self.angle)
        dir_val = 0 if angle >= 0 else  1
        self.move(dir_val, steps)
        self.angle = angle
        

    def increase_angle(self, angle: float) -> None:
        #self.set_angle(self.angle + angle)
        pass


    def move(self, dir_val: int, steps: int) -> None:
        GPIO.output(self.dir_pin, dir_val)

        for _ in range(steps):
            # 500 us square pulse
            GPIO.output(self.step_pin, 1)
            time.sleep(0.0005)
            GPIO.output(self.step_pin, 0)

            # Delay to control speed
            delay = 0.001
            time.sleep(delay)


    def degrees_to_steps(self, degrees: float) -> int:
        steps = int((abs(degrees) * Stepper.steps_per_revolution * self.ustep)/360)
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


