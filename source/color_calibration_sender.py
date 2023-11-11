import cv2
import socket
import struct
import time
import numpy as np

host = "192.168.100.192"
port = 8080

print(f"Trying to stablish connection with {host}:{port}")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((host, port))

print("Connection successful, starting video stream")
cap = cv2.VideoCapture(0)  # Use 0 for the default camera or specify the video file path

counter = 0
start = time.time()

# Defining initial threshold values
low_vals = np.array([0, 0, 0])
high_vals = np.array([255, 255, 255])

while True:
    ret, frame = cap.read()

    # Changing frame to hsv
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Applying color threshold filter
    frame_mask = cv2.inRange(hsv_frame, low_vals, high_vals)
    frame = cv2.bitwise_and(frame, frame, mask=frame_mask)

    # Serialize the image
    frame_data = cv2.imencode('.jpg', frame)[1]
    frame_data = frame_data.tobytes()

    # Pack it along with the image size and send it
    frame_size = struct.pack("!L", len(frame_data))    
    sock.sendall(frame_size + frame_data)

    # Read the bytes for the threshold values
    low_vals_data = sock.recv(3)
    high_vals_data = sock.recv(3)
    low_vals = np.frombuffer(low_vals_data, dtype=np.uint8)
    high_vals = np.frombuffer(high_vals_data, dtype=np.uint8)
    print(low_vals, high_vals)
    
    counter += 1
    if (counter % 20 == 0):
        fps = counter/(time.time() - start)
        print(f"fps: {fps}")
        counter = 0
        start = time.time()

# Release the video capture and close the socket when done
cap.release()
sock.close()