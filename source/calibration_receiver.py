import cv2
import socket
import struct
import numpy as np
import json


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


def on_min_val_trackbar(val):
    global min_val
    min_val = val


def on_max_val_trackbar(val):
    global max_val
    max_val = val


def stablish_connection(host, port):
    receiver_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    receiver_socket.bind((host, port))
    receiver_socket.listen(1)
    print(f"Listening for connection on port {port}")

    # Accept a connection from the sender
    sender_socket, sender_address = receiver_socket.accept()
    print(f"Received connection from {sender_address}")
    return sender_socket


def create_color_calibration_window(window_name, colorspace):
    cv2.namedWindow(window_name)
    for i, channel in enumerate(colorspace):
        cv2.createTrackbar(f'Low {channel}', window_name, low_vals[i], max_vals[i], globals()[f'on_low_{i}_thresh_trackbar'])
        cv2.createTrackbar(f'High {channel}', window_name, high_vals[i], max_vals[i], globals()[f'on_high_{i}_thresh_trackbar'])


def create_edge_calibration_window(window_name):
    cv2.namedWindow(window_name)
    cv2.createTrackbar(f'Min', window_name, min_val, 255, on_min_val_trackbar)
    cv2.createTrackbar(f'Max', window_name, max_val, 255, on_max_val_trackbar)


def load_calibration_params():
    param_file = 'assets/calibration_params.json'
    with open(param_file, 'r') as file:
        params = json.loads(file.read())
    return params


def save_calibration_params(params):
    param_file = 'assets/calibration_params.json'
    with open(param_file, 'w') as file:
        params_json = json.dumps(params, indent=4)
        file.write(params_json)


if __name__ == "__main__":

    host = "0.0.0.0"
    port = 8080
    sock = stablish_connection(host, port)

    data = b""
    payload_size = struct.calcsize("!L")


    params = load_calibration_params()

    # Defining color mask initial values
    global low_vals, high_vals, max_vals
    low_vals = params['base_color']['low']
    max_vals = np.array([255, 255, 255], dtype=np.uint8)
    create_color_calibration_window('color_calibration', 'LAB')

    # Defining edge initial values
    min_val = 50
    max_val = 150
    create_edge_calibration_window('edge_calibration')    

    while True:
        while len(data) < payload_size:
            data += sock.recv(4096)
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack("!L", packed_msg_size)[0]

        while len(data) < msg_size:
            data += sock.recv(4096)
        frame_data = data[:msg_size]
        data = data[msg_size:]

        # Convert the frame data back to a NumPy array
        frame = np.frombuffer(frame_data, dtype=np.uint8)
        frame = cv2.imdecode(frame, 1)

        # Chanding to HSV and filtering
        blurred_frame = cv2.GaussianBlur(frame, (5,5), 0)
        lab_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2LAB)

        # Applying color threshold filter
        mask = cv2.inRange(lab_frame, low_vals, high_vals)
        masked_frame = cv2.bitwise_and(frame, frame, mask=mask)

        # Extracting edges form the mask
        blurred = cv2.GaussianBlur(mask, (15,15), 0)
        edges = cv2.Canny(blurred, min_val, max_val)

        # Getting the contours and drawing them
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            epsilon = 0.001 * cv2.arcLength(contour, True)
            approx_poly = cv2.approxPolyDP(contour, epsilon, True)
            cv2.drawContours(frame, [approx_poly], 0, [0,255,0], 2)

        # Display the received frame
        cv2.imshow('edge_calibration', edges)
        cv2.imshow('color_calibration', masked_frame)
        cv2.imshow('polygon', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video window and close the sockets when done
    cv2.destroyAllWindows()
    sock.close()