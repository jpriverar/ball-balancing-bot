import cv2
import socket
import struct
import time
from picamera2 import Picamera2
import select
import numpy as np
from manipulator import RRSManipulator
from utils import load_calibration_params

def init_camera_with_opencv():
    cap = cv2.VideoCapture(0)  # Use 0 for the default camera or specify the video file path
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_FPS, 30)
    return cap


def init_camera_with_picamera2():
    cap = Picamera2()
    config = cap.create_preview_configuration(main={"size":(1920, 1080), "format":"RGB888"}, controls={"FrameRate":90})
    cap.configure(config)
    cap.start()
    time.sleep(3)
    return cap

params = load_calibration_params()['manipulator']
bot = RRSManipulator(params['base_radius'],
                     params['platform_radius'],
                     params['low_arm_length'],
                     params['high_arm_length'])

bot.calibrate()
bot.home()

# Stream socket connection
host = "192.168.100.192"
port = 8080

print(f"Starting streaming to {host}:{port}")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Camera setup
cap = init_camera_with_opencv()

frame_counter = 0
start = time.time()
while True:

    # Capture and send the image
    ret, frame = cap.read()
    if not ret: 
        print("Something went wrong: could not capture frame...")
        break

    # Serialize the image and send it
    _, frame_data = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    sock.sendto(frame_data.tobytes(), (host, port))

    # Read control messages if any
    ready, _, _ = select.select([sock], [], [], 0)
    if ready:
        data_raw = sock.recvfrom(512)[0]
        data = np.frombuffer(data_raw, dtype=np.float32)
        print(data)
        bot.move_pose(data[0], data[1], -data[2]) 

# Release the video capture and close the socket when done
cap.release()
sock.close()