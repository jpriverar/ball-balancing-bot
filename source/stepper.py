import RPi.GPIO as GPIO
import time
import numpy as np

class Stepper:
    STEPS_PER_REV: int = 200

    def __init__(self, step_pin: int, dir_pin: int, ms1_pin: int, ms2_pin: int, starting_angle: float = 0) -> None:
        self.__step_pin = step_pin
        self.__dir_pin = dir_pin
        self.__ms1 = ms1_pin
        self.__ms2 = ms2_pin

        GPIO.setup(self.__step_pin, GPIO.OUT)
        GPIO.setup(self.__dir_pin, GPIO.OUT)
        GPIO.setup(self.__ms1, GPIO.OUT)
        GPIO.setup(self.__ms2, GPIO.OUT)

        # 16 microstepping by default - maximum precission and quiet
        self.set_ustep(16)

        # Setting default current and target pos, speed and acceleration
        self.__curr_pos = 0
        self.__target_pos = 0 
        self.__speed = 0.0
        self.__accel = 1.0
        self.__step_interval = 0.1
        self.__last_step_time = 0
        self.__max_speed = 1.0

    # Getters

    @property
    def distance_to_go(self) -> int:
        return self.__target_pos - self.__curr_pos
    

    @property
    def target_position(self) -> int:
        return self.__target_pos
    

    @property
    def current_position(self) -> int:
        return self.__curr_pos
    

    @property
    def speed(self) -> float:
        return self.__speed
    
    @property
    def delay(self) -> float:
        return self.__step_interval
    
    # Setters

    def set_ustep(self, ustep: int) -> None:
        if ustep not in [2,4,8,16]: 
            print('WARNING: unvalid microstepping passed:', ustep)
            return
        if ustep == 2:
            GPIO.output(self.__ms1, 1)
            GPIO.output(self.__ms2, 0)
        elif ustep == 4:
            GPIO.output(self.__ms1, 0)
            GPIO.output(self.__ms2, 1)
        elif ustep == 8:
            GPIO.output(self.__ms1, 0)
            GPIO.output(self.__ms2, 0)
        elif ustep == 16:
            GPIO.output(self.__ms1, 1)
            GPIO.output(self.__ms2, 2)
        self.ustep = ustep

    
    def set_current_position(self, position: int) -> None:
        self.__target_pos = self.__curr_pos = position
        self.__step_interval = 0.0
        self.__speed = 0.0

    
    def set_max_speed(self, speed: float) -> None:
        self.__max_speed = max(1.0, speed)
        self.compute_new_speed()


    def set_acceleration(self, accel: float) -> None:
        self.__accel = max(1.0, accel)
        self.compute_new_speed()


    def set_speed(self, speed: float) -> None:
        if speed >= 0:
            GPIO.output(self.__dir_pin, 0)
        else:
            GPIO.output(self.__dir_pin, 1)

        self.__speed = speed
        if speed != 0:
            self.__step_interval = abs(1 / self.__speed)    

    # Methods    

    def move_to(self, absolute: int) -> None:
        if self.__target_pos != absolute:
            self.__target_pos = absolute
            self.compute_new_speed()


    def move(self, relative: int) -> None:
        self.move_to(self.__target_pos + relative)


    def compute_new_speed(self) -> None:
        new_speed = self.desired_speed()
        self.set_speed(new_speed)


    def desired_speed(self) -> float:
        distance = self.distance_to_go

        if distance == 0:
            return 0.0 # We're already there
        elif distance > 0:
            required_speed = np.sqrt(2.0 * distance * self.__accel)
        else: 
            required_speed = -np.sqrt(2.0 * -distance * self.__accel)

        # Need to accelerate clockwise
        if required_speed > self.__speed:
            if self.__speed == 0:
                required_speed = np.sqrt(2.0 * self.__accel)
            else:
                required_speed = min(self.__max_speed, self.__speed + abs(self.__accel / self.__speed))

        # Need to accelerate counter clockwise
        elif required_speed < self.__speed:
            if self.__speed == 0:
                required_speed = -np.sqrt(2.0 * self.__accel)
            else:
                required_speed = max(-self.__max_speed, self.__speed - abs(self.__accel / self.__speed))

        return required_speed
    

    def run_speed(self) -> bool:
        start = time.time()
        if (start - self.__last_step_time >= self.__step_interval):
            if (self.__speed > 0):
                self.__curr_pos += 1
            else:
                self.__curr_pos -= 1
            self.step()
            self.__last_step_time = start
            return True
        return False
    

    def run(self) -> bool:
        if (self.__target_pos == self.__curr_pos):
            return False
        
        if self.run_speed():
            self.compute_new_speed()
        return True


    def step(self) -> None:
        GPIO.output(self.__step_pin, 1)
        time.sleep(0.000001)
        GPIO.output(self.__step_pin, 0)
        time.sleep(0.000001)


    def degrees_to_steps(self, degrees: float) -> int:
        steps = int((degrees * Stepper.STEPS_PER_REV * self.ustep)/360)
        return steps
    
    
    def steps_to_degrees(self, steps: int) -> float:
        degrees = (steps * 360)/(Stepper.STEPS_PER_REV * self.ustep)
        return degrees

    
if __name__ == '__main__':

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    stepper1 = Stepper(23, 18, 25, 24)
    stepper2 = Stepper(27, 22, 4, 17)
    stepper3 = Stepper(5, 6, 19, 13)

    steppers = (stepper1, stepper2, stepper3)

    stepper1.move_to(-300)
    stepper1.set_max_speed(1000)
    stepper1.set_acceleration(500)

    stepper2.move_to(-100)
    stepper2.set_max_speed(100)
    stepper2.set_acceleration(100)

    stepper3.move_to(-400)
    stepper3.set_max_speed(500)
    stepper3.set_acceleration(500)

    while True:
        if stepper1.distance_to_go == 0:
            stepper1.move_to(-stepper1.current_position)

        if stepper2.distance_to_go == 0:
            stepper2.move_to(-stepper2.current_position)

        if stepper3.distance_to_go == 0:
            stepper3.move_to(-stepper3.current_position)

        stepper1.run()
        stepper2.run()
        stepper3.run()
