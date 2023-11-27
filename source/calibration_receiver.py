import cv2
import socket
import struct
import numpy as np
import json


def on_platform_low_0_thresh_trackbar(val):
    global platform_low_vals
    platform_low_vals[0] = val


def on_platform_high_0_thresh_trackbar(val):
    global platform_high_vals
    platform_high_vals[0] = val
    

def on_platform_low_1_thresh_trackbar(val):
    global platform_low_vals
    platform_low_vals[1] = val


def on_platform_high_1_thresh_trackbar(val):
    global platform_high_vals
    platform_high_vals[1] = val
    

def on_platform_low_2_thresh_trackbar(val):
    global platform_low_vals
    platform_low_vals[2] = val
    

def on_platform_high_2_thresh_trackbar(val):
    global platform_high_vals
    platform_high_vals[2] = val


def on_ball_low_0_thresh_trackbar(val):
    global ball_low_vals
    ball_low_vals[0] = val


def on_ball_high_0_thresh_trackbar(val):
    global ball_high_vals
    ball_high_vals[0] = val
    

def on_ball_low_1_thresh_trackbar(val):
    global ball_low_vals
    ball_low_vals[1] = val


def on_ball_high_1_thresh_trackbar(val):
    global ball_high_vals
    ball_high_vals[1] = val
    

def on_ball_low_2_thresh_trackbar(val):
    global ball_low_vals
    ball_low_vals[2] = val
    

def on_ball_high_2_thresh_trackbar(val):
    global ball_high_vals
    ball_high_vals[2] = val


def on_min_val_trackbar(val):
    global min_val
    min_val = val


def on_max_val_trackbar(val):
    global max_val
    max_val = val


def on_yaw_val_trackbar(val):
    global platform_yaw
    platform_yaw = val


def listen_for_connection(host, port):
    receiver_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver_socket.bind((host, port))
    print(f"Listening for connection on port {port}")
    return receiver_socket


def create_platform_color_calibration_window(window_name, colorspace):
    cv2.namedWindow(window_name)
    for i, channel in enumerate(colorspace):
        cv2.createTrackbar(f'Low {channel}', window_name, platform_low_vals[i], platform_max_vals[i], globals()[f'on_platform_low_{i}_thresh_trackbar'])
        cv2.createTrackbar(f'High {channel}', window_name, platform_high_vals[i], platform_max_vals[i], globals()[f'on_platform_high_{i}_thresh_trackbar'])


def create_ball_color_calibration_window(window_name, colorspace):
    cv2.namedWindow(window_name)
    for i, channel in enumerate(colorspace):
        cv2.createTrackbar(f'Low {channel}', window_name, ball_low_vals[i], ball_max_vals[i], globals()[f'on_ball_low_{i}_thresh_trackbar'])
        cv2.createTrackbar(f'High {channel}', window_name, ball_high_vals[i], ball_max_vals[i], globals()[f'on_ball_high_{i}_thresh_trackbar']) 


def create_edge_calibration_window(window_name):
    cv2.namedWindow(window_name)
    cv2.createTrackbar('Min', window_name, min_val, 255, on_min_val_trackbar)
    cv2.createTrackbar('Max', window_name, max_val, 255, on_max_val_trackbar)


def create_yaw_calibration_window(window_name):
    cv2.namedWindow(window_name)
    cv2.createTrackbar('Yaw', window_name, platform_yaw, 180, on_yaw_val_trackbar)


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
        cv2.line(frame, center, (x,y), (255,255,255), 3)


def get_platform_axes(vertices):
    x1 = None
    x2 = None
    x_axis_point = None
    y_axis_point = None
    for vertex in vertices:
        if y_axis_point is None or vertex[1] > y_axis_point[1]:
            y_axis_point = vertex

        if x1 is None or vertex[0] < x1[0]:
            x1 = vertex

        elif x2 is None or vertex[0] < x2[0]:
            x2 = vertex
    
    x_axis_point = ((x2[0]+x1[0])//2, (x2[1]+x1[1])//2)
    return [x_axis_point, y_axis_point]


if __name__ == "__main__":

    host = "0.0.0.0"
    port = 8080
    sock = listen_for_connection(host, port)

    data = b""
    max_datagram_size = 65536
    payload_size = struct.calcsize("!L")

    params = load_calibration_params()

    # Defining platform color mask initial values
    platform_low_vals = np.array(params['color_thresholds']['platform']['low'], dtype=np.uint8)
    platform_high_vals = np.array(params['color_thresholds']['platform']['high'], dtype=np.uint8)
    platform_max_vals = np.array([255,255,255])
    create_platform_color_calibration_window('platform_calibration', 'LAB')
    
    # Defining ball color mask initial values
    ball_low_vals = np.array(params['color_thresholds']['ball']['low'], dtype=np.uint8)
    ball_high_vals = np.array(params['color_thresholds']['ball']['high'], dtype=np.uint8)
    ball_max_vals = np.array([255,255,255])
    create_ball_color_calibration_window('ball_calibration', 'LAB')

    # Defining edge initial values
    min_val = params['edge_detection']['thresh_min']
    max_val = params['edge_detection']['thresh_max']
    create_edge_calibration_window('edge_calibration') 

    # Defining platform yaw initial value
    platform_yaw = params['platform_yaw'] 
    create_yaw_calibration_window('yaw_calibration')

    while True:
        frame_data = sock.recvfrom(max_datagram_size)[0]

        # Convert the frame data back to a NumPy array
        frame_raw = np.frombuffer(frame_data, dtype=np.uint8)
        frame = cv2.imdecode(frame_raw, 1)

        # Changing to HSV and filtering
        blurred_frame = cv2.GaussianBlur(frame, (5,5), 0)
        lab_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2LAB)

        # Applying platform color threshold filter
        platform_mask = cv2.inRange(lab_frame, platform_low_vals, platform_high_vals)
        platform_masked_frame = cv2.bitwise_and(frame, frame, mask=platform_mask)

        # Applying ball color threshold filter
        ball_mask = cv2.inRange(lab_frame, ball_low_vals, ball_high_vals)
        ball_masked_frame = cv2.bitwise_and(frame, frame, mask=ball_mask)

        # Extracting edges form the masks
        blurred_platform = cv2.GaussianBlur(platform_mask, (5,5), 0)
        blurred_ball = cv2.GaussianBlur(ball_mask, (5,5), 0)
        platform_edges = cv2.Canny(blurred_platform, min_val, max_val)
        ball_edges = cv2.Canny(blurred_ball, min_val, max_val)

        # Getting the platform contours and drawing them
        geo_frame = frame.copy()
        contours, _ = cv2.findContours(platform_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            contour = sorted(contours, key=cv2.contourArea, reverse=True)[0].squeeze()
            hull = cv2.convexHull(contour)
            cv2.drawContours(geo_frame, [hull], 0, (0,255,0), 2)

            # Drawing the convex hull contour to extract lines
            hull_frame = np.zeros_like(platform_mask)
            cv2.drawContours(hull_frame, [hull], 0, 255, 3)

            center, radius = cv2.minEnclosingCircle(hull)
            center = tuple(map(int, center))
            radius = int(radius)
            cv2.circle(geo_frame, center, radius, (0,0,255), 2)

            line_frame = np.zeros_like(platform_mask)
            for angle in np.linspace(0,300,6):
                draw_polar_line(line_frame, center, radius, angle + platform_yaw)
                draw_polar_line(geo_frame, center, radius, angle + platform_yaw)

            intersections = cv2.bitwise_and(hull_frame, line_frame)
            intersection_contours, _ = cv2.findContours(intersections, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            intersection_points = []
            for cont in intersection_contours:
                cx, cy = get_contour_centroid(cont)
                intersection_points.append((cx,cy))
                cv2.circle(frame, (cx, cy), 3, (0,0,255), -1)

            center = get_contour_centroid(hull)
            x_point, y_point = get_platform_axes(intersection_points)
            cv2.circle(frame, center, 3, (0,0,255), -1)
            cv2.line(frame, center, y_point, (0,255,0), 2)
            cv2.line(frame, center, x_point, (0,255,255), 2)

        contours, _ = cv2.findContours(ball_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            contour = sorted(contours, key=cv2.contourArea, reverse=True)[0].squeeze()
            cv2.drawContours(geo_frame, [contour], 0, (0,255,0), 2)

            ball_center = get_contour_centroid(contour)
            cv2.circle(frame, ball_center, 3, (0,0,255), -1)
            
        
        # Display the received frame
        cv2.imshow('edge_calibration', platform_edges)
        cv2.imshow('platform_calibration', platform_masked_frame)
        cv2.imshow('ball_calibration', ball_masked_frame)
        cv2.imshow('yaw_calibration', geo_frame)
        cv2.imshow('final', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video window and close the sockets when done
    cv2.destroyAllWindows()
    sock.close()

    # Updating params with ending values
    params['color_thresholds']['platform']['low'] = platform_low_vals.tolist()
    params['color_thresholds']['platform']['high'] = platform_high_vals.tolist()
    params['color_thresholds']['ball']['low'] = ball_low_vals.tolist()
    params['color_thresholds']['ball']['high'] = ball_high_vals.tolist()
    params['edge_detection']['thresh_min'] = min_val
    params['edge_detection']['thresh_max'] = max_val
    params['platform_yaw'] = platform_yaw
    save_calibration_params(params)