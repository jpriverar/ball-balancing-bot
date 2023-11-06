import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

# Pin definition
dir1 = 18
step1 = 23
ms1_1 = 25
ms2_1 = 24

dir2 = 22
step2 = 27
ms1_2 = 4
ms2_2 = 17

dir3 = 6
step3 = 5
ms1_3 = 19
ms2_3 = 13


# Pin mode -> 1:output, 0:input
GPIO.setup(dir1, GPIO.OUT)
GPIO.setup(dir2, GPIO.OUT)
GPIO.setup(dir3, GPIO.OUT)
GPIO.setup(step1, GPIO.OUT)
GPIO.setup(step2, GPIO.OUT)
GPIO.setup(step3, GPIO.OUT)
GPIO.setup(ms1_1, GPIO.OUT)
GPIO.setup(ms2_1, GPIO.OUT)
GPIO.setup(ms1_2, GPIO.OUT)
GPIO.setup(ms2_2, GPIO.OUT)
GPIO.setup(ms1_3, GPIO.OUT)
GPIO.setup(ms2_3, GPIO.OUT)

# Choosing step size
GPIO.output(ms1_1, 1)
GPIO.output(ms2_1, 1)
GPIO.output(ms1_2, 1)
GPIO.output(ms2_2, 1)
GPIO.output(ms1_3, 1)
GPIO.output(ms2_3, 1)

steps_per_revolution = 200
angle = 20 # degrees
steps = int((angle * steps_per_revolution * 16)/360)
print(f"Doing {steps} steps")

dir_val = 1
while True:
    # Setting a rotation direction
    dir_val ^= 1 
    GPIO.output(dir1, dir_val)
    GPIO.output(dir2, dir_val)
    GPIO.output(dir3, dir_val)

    count = 0
    while count < steps:
        GPIO.output(step1, 1)
        GPIO.output(step2, 1)
        GPIO.output(step3, 1)
        time.sleep(0.0005)
        GPIO.output(step1, 0)
        GPIO.output(step2, 0)
        GPIO.output(step3, 0)
        time.sleep(0.0005)

        time.sleep(0.001)
        count += 1
        print(count)
        
