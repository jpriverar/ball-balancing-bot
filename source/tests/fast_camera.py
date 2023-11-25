from picamera2 import Picamera2
from pprint import *
import time

cam = Picamera2()
config = cam.create_preview_configuration(main={'size': (640, 480), 'format':'RGB888'}, controls={'FrameRate':30})
pprint(cam.sensor_modes)
cam.configure(config)
cam.start()
start = time.time()

count = 0
fps = 0
while True:
    frame = cam.capture_array('main')
    count += 1

    if count % 20 == 0:
        fps = count/(time.time()-start)
        count = 0
        print('fps:', fps)
        start = time.time()

