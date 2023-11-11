import cv2
import numpy as np
import struct
import socket


def on_low_0_thresh_trackbar(val):
    global low_vals
    low_vals[0] = val


def on_high_0_thresh_trackbar(val):
    global high_vals
    high_vals[0] = val
    

def on_low_1_thresh_trackbar(val):
    global low_vals
    low_vals[1] = val


def on_high_1_thresh_trackbar(val):
    global high_vals
    high_vals[1] = val
    

def on_low_2_thresh_trackbar(val):
    global low_vals
    low_vals[2] = val
    

def on_high_2_thresh_trackbar(val):
    global high_vals
    high_vals[2] = val
    

# Starting connection with RPi
host = "0.0.0.0"
port = 8080

receiver_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
receiver_socket.bind((host, port))
receiver_socket.listen(1)
print(f"Listening for connection on port {port}")

# Accept a connection from the sender
sender_socket, sender_address = receiver_socket.accept()
print(f"Received connection from {sender_address}")
    
data = b""
payload_size = struct.calcsize("!L")

# Defining initial values
global low_vals, high_vals, max_vals
low_vals = np.array([0, 0, 0], dtype=np.uint8)
high_vals = np.array([180, 255, 255], dtype=np.uint8)
max_vals = np.array([180, 255, 255], dtype=np.uint8)

# Creating named window
window_name = 'Color Calibrator'
cv2.namedWindow(window_name)

# Creating trackbars
for i, channel in enumerate('HSV'):
    cv2.createTrackbar(f'Low {channel}', window_name, low_vals[i], max_vals[i], globals()[f'on_low_{i}_thresh_trackbar'])
    cv2.createTrackbar(f'High {channel}', window_name, high_vals[i], max_vals[i], globals()[f'on_high_{i}_thresh_trackbar'])

while True:
    # Reading bytes until full payload size received
    while len(data) < payload_size:
        data += sender_socket.recv(4096)
    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack("!L", packed_msg_size)[0]

    # Reading bytes until full msg size received
    while len(data) < msg_size:
        data += sender_socket.recv(4096)
    frame_data = data[:msg_size]
    data = data[msg_size:]

    # Sending the threshold values
    sender_socket.sendall(low_vals.tobytes())
    sender_socket.sendall(high_vals.tobytes())

    # Convert the serialized data into frame
    frame = np.frombuffer(frame_data, dtype=np.uint8)    
    frame = cv2.imdecode(frame, 1) # 1 for color image
    
    # # Display the received frame
    cv2.imshow(window_name, frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video window and close the sockets when done
cv2.destroyAllWindows()
receiver_socket.close()
sender_socket.close()