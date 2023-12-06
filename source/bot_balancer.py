import cv2
import socket
import struct
import time
from picamera2 import Picamera2
import select
import numpy as np
from manipulator import RRSManipulator

def init_camera_with_opencv():
    cap = cv2.VideoCapture(0)  # Use 0 for the default camera or specify the video file path
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_FPS, 30)
    return cap


def init_camera_with_picamera2():
    cap = Picamera2()
    config = cap.create_preview_configuration(main={"size":(1920, 1080), "format":"RGB888"}, controls={"FrameRate":30})
    cap.configure(config)
    cap.start()
    time.sleep(3)
    return cap


# Stream socket connection
host = "192.168.100.192"
port = 8080

print(f"Starting streaming to {host}:{port}")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Camera setup
cap = init_camera_with_opencv()

counter = 0
start = time.time()
while True:
    ret, frame = cap.read()
    #frame = cap.capture_array('main')
    #frame = frame[80:420,100:500,:] # Taking only the roi

    # Serialize the image
    _, frame_data = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    frame_data = frame_data.tobytes()

    # Pack it along with the image size
    #frame_size = struct.pack("!L", len(frame_data))    
    sock.sendto(frame_data, (host, port))

    ready, _, _ = select.select([sock], [], [], 0)
    if ready:
        data_raw = sock.recvfrom(512)[0]
        data = np.frombuffer(data_raw, dtype=np.float64)
        print(data)
    
    counter += 1
    if (counter % 20 == 0):
        fps = counter/(time.time() - start)
        print(f"fps: {fps}")
        counter = 0
        start = time.time()

# Release the video capture and close the socket when done
cap.release()
sock.close()
