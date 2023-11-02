import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

# Pin definition
dir1 = 22
step1 = 27

dir2 = 9
step2 = 10

dir3 = 25
step3 = 11

ms1 = 23
ms2 = 24
ms3 = 4

# Pin mode -> 1:output, 0:input
GPIO.setup(dir1, GPIO.OUT)
GPIO.setup(dir2, GPIO.OUT)
GPIO.setup(dir3, GPIO.OUT)
GPIO.setup(step1, GPIO.OUT)
GPIO.setup(step2, GPIO.OUT)
GPIO.setup(step3, GPIO.OUT)
GPIO.setup(ms1, GPIO.OUT)
GPIO.setup(ms2, GPIO.OUT)
GPIO.setup(ms3, GPIO.OUT)

# Choosing step size
GPIO.output(ms1, 1)
GPIO.output(ms2, 1)
GPIO.output(ms3, 1)


steps_per_revolution = 200
angle = 20 # degrees
steps = int((angle * steps_per_revolution * 16)/360)
print(f"Doing {steps} steps")

dir_val = 1
while dir_val == 1:
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

        #time.sleep(0.001)
        count += 1
        print(count)
        
