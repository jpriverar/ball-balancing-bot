import cv2
import socket
import struct

host = "192.168.100.10"
port = 8080

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((host, port))

# Open a video capture object using OpenCV
cap = cv2.VideoCapture(0)  # Use 0 for the default camera or specify the video file path

while True:
    ret, frame = cap.read()
    
    # Serialize the image
    frame_data = cv2.imencode('.jpg', frame)[1]
    frame_data = frame_data.tobytes()

    # Pack it along with the image size
    frame_size = struct.pack("!L", len(frame_data))    
    sock.sendall(frame_size + frame_data)

# Release the video capture and close the socket when done
cap.release()
sock.close()