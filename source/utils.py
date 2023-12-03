import socket
import json
import cv2
import numpy as np


def listen_for_connection(host, port):
    receiver_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver_socket.bind((host, port))
    print(f"Listening for connection on port {port}")
    return receiver_socket


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
    print("Calibration parameters saved successfully")


def get_contour_centroid(contour):
    M = cv2.moments(contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        cx, cy = 0, 0
    return cx, cy


def draw_polar_line(frame, center, rho, theta):
        x = center[0] + int(rho * np.cos(np.radians(theta)))
        y = center[1] - int(rho * np.sin(np.radians(theta)))
        cv2.line(frame, center, (x,y), (255,255,255), 3, lineType=cv2.LINE_AA)