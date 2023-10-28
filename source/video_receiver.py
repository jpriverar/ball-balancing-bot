import cv2
import socket
import struct
import numpy as np

host = "localhost"
port = 8080

receiver_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
receiver_socket.bind((host, port))
receiver_socket.listen(1)

# Accept a connection from the sender
sender_socket, sender_address = receiver_socket.accept()

data = b""
payload_size = struct.calcsize("!L")

while True:
    while len(data) < payload_size:
        data += sender_socket.recv(4096)
    
    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack("!L", packed_msg_size)[0]

    while len(data) < msg_size:
        data += sender_socket.recv(4096)

    frame_data = data[:msg_size]
    data = data[msg_size]

    # Convert the frame data back to a NumPy array
    frame = np.frombuffer(frame_data, dtype=np.uint8)
    
    # Decode the frame
    frame = cv2.imdecode(frame, 1)
    
    # Display the received frame
    cv2.imshow('Received Video', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video window and close the sockets when done
cv2.destroyAllWindows()
receiver_socket.close()
sender_socket.close()