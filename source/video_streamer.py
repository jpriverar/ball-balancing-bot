import cv2
import socket
import struct
import time

host = "192.168.100.192"
port = 8080

print(f"Trying to stablish connection with {host}:{port}")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((host, port))

print("Connection successful, starting video stream")
cap = cv2.VideoCapture(0)  # Use 0 for the default camera or specify the video file path

counter = 0
start = time.time()

while True:
    ret, frame = cap.read()

    frame = cv2.resize(frame, (int(frame.shape[1]*0.80), int(frame.shape[0]*0.80)))

    # Serialize the image
    frame_data = cv2.imencode('.jpg', frame)[1]
    frame_data = frame_data.tobytes()

    # Pack it along with the image size
    frame_size = struct.pack("!L", len(frame_data))    
    sock.sendall(frame_size + frame_data)
    
    counter += 1
    if (counter % 20 == 0):
        fps = counter/(time.time() - start)
        print(f"fps: {fps}")
        counter = 0
        start = time.time()

# Release the video capture and close the socket when done
cap.release()
sock.close()