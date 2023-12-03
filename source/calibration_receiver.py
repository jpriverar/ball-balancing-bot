import cv2
import numpy as np
import time
from utils import *


def on_platform_brightness_trackbar(val):
    global platform_brightness
    platform_brightness = val


def on_platform_contrast_trackbar(val):
    global platform_contrast
    platform_contrast = val


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


def on_ball_brightness_trackbar(val):
    global ball_brightness
    ball_brightness = val


def on_ball_contrast_trackbar(val):
    global ball_contrast
    ball_contrast = val


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


def on_rotation_val_trackbar(val):
    global platform_rotation
    platform_rotation = val


def create_platform_color_calibration_window(window_name, colorspace):
    cv2.namedWindow(window_name)
    cv2.createTrackbar(f'Brightness', window_name, platform_brightness, 100, on_platform_brightness_trackbar)
    cv2.setTrackbarMin(f'Brightness', window_name, -100)
    cv2.createTrackbar(f'Contrast', window_name, platform_contrast, 2, on_platform_contrast_trackbar)
    cv2.setTrackbarMin(f'Contrast', window_name, -5)
    for i, channel in enumerate(colorspace):
        cv2.createTrackbar(f'Low {channel}', window_name, platform_low_vals[i], 255, globals()[f'on_platform_low_{i}_thresh_trackbar'])
        cv2.createTrackbar(f'High {channel}', window_name, platform_high_vals[i], 255, globals()[f'on_platform_high_{i}_thresh_trackbar'])


def create_ball_color_calibration_window(window_name, colorspace):
    cv2.namedWindow(window_name)
    cv2.createTrackbar(f'Brightness', window_name, ball_brightness, 100, on_ball_brightness_trackbar)
    cv2.setTrackbarMin(f'Brightness', window_name, -100)
    cv2.createTrackbar(f'Contrast', window_name, ball_contrast, 2, on_ball_contrast_trackbar)
    cv2.setTrackbarMin(f'Contrast', window_name, -5)
    for i, channel in enumerate(colorspace):
        cv2.createTrackbar(f'Low {channel}', window_name, ball_low_vals[i], 255, globals()[f'on_ball_low_{i}_thresh_trackbar'])
        cv2.createTrackbar(f'High {channel}', window_name, ball_high_vals[i], 255, globals()[f'on_ball_high_{i}_thresh_trackbar']) 


def create_rotation_calibration_window(window_name):
    cv2.namedWindow(window_name)
    cv2.createTrackbar('Rotation', window_name, platform_rotation, 180, on_rotation_val_trackbar)


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

    params = load_calibration_params()

    # Defining platform color mask initial values
    platform_low_vals = np.array(params['platform']['low_threshold'], dtype=np.uint8)
    platform_high_vals = np.array(params['platform']['high_threshold'], dtype=np.uint8)
    platform_brightness = params['platform']['brightness']
    platform_contrast = params['platform']['contrast']
    create_platform_color_calibration_window('platform_calibration', 'LAB')
    
    # Defining ball color mask initial values
    ball_low_vals = np.array(params['ball']['low_threshold'], dtype=np.uint8)
    ball_high_vals = np.array(params['ball']['high_threshold'], dtype=np.uint8)
    ball_brightness = params['ball']['brightness']
    ball_contrast = params['ball']['contrast']
    create_ball_color_calibration_window('ball_calibration', 'LAB')

    # Defining platform rotation initial value
    platform_rotation = params['platform']['rotation'] 
    create_rotation_calibration_window('rotation_calibration')

    count = 0
    start = time.time()
    while True:
        frame_data = sock.recvfrom(65536)[0]

        # Convert the frame data back to a NumPy array
        frame_raw = np.frombuffer(frame_data, dtype=np.uint8)
        frame = cv2.imdecode(frame_raw, 1)

        platform_frame = frame.copy()
        ball_frame = frame.copy()

        # Enhancing brigthness
        platform_frame = np.uint8(np.clip(np.int32(platform_frame) + platform_brightness, 0, 255))
        ball_frame = np.uint8(np.clip(np.int32(ball_frame) + ball_brightness, 0, 255))

        # Enhancing contrast
        platform_frame = np.uint8(np.clip(platform_frame * platform_contrast, 0, 255))
        ball_frame = np.uint8(np.clip(ball_frame * ball_contrast, 0, 255))

        # Changing to HSV and filtering
        blurred_platform = cv2.GaussianBlur(platform_frame, (5,5), 0)
        lab_platform = cv2.cvtColor(blurred_platform, cv2.COLOR_BGR2LAB)

        blurred_ball = cv2.GaussianBlur(ball_frame, (5,5), 0)
        lab_ball = cv2.cvtColor(blurred_ball, cv2.COLOR_BGR2LAB)

        # Applying platform color threshold filter
        platform_mask = cv2.inRange(lab_platform, platform_low_vals, platform_high_vals)
        platform_masked_frame = cv2.bitwise_and(platform_frame, platform_frame, mask=platform_mask)

        # Applying ball color threshold filter
        ball_mask = cv2.inRange(lab_ball, ball_low_vals, ball_high_vals)
        ball_masked_frame = cv2.bitwise_and(platform_frame, frame, mask=ball_mask)

        # Extracting edges form the masks
        blurred_platform = cv2.GaussianBlur(platform_mask, (5,5), 0)
        blurred_ball = cv2.GaussianBlur(ball_mask, (5,5), 0)
        platform_edges = cv2.Canny(blurred_platform, 150, 255)
        ball_edges = cv2.Canny(blurred_ball, 150, 255)

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
            cv2.circle(geo_frame, center, radius, (0,0,255), 2, lineType=cv2.LINE_AA)

            line_frame = np.zeros_like(platform_mask)
            for angle in np.linspace(0,300,6):
                draw_polar_line(line_frame, center, radius, angle + platform_rotation)
                draw_polar_line(geo_frame, center, radius, angle + platform_rotation)

            intersections = cv2.bitwise_and(hull_frame, line_frame)
            intersection_contours, _ = cv2.findContours(intersections, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            intersection_points = []
            for cont in intersection_contours:
                cx, cy = get_contour_centroid(cont)
                intersection_points.append((cx,cy))
                cv2.circle(frame, (cx, cy), 3, (0,0,255), -1)

            center = get_contour_centroid(hull)
            #x_point, y_point = get_platform_axes(intersection_points)
            cv2.circle(frame, center, 3, (0,0,255), -1, lineType=cv2.LINE_AA)
            #cv2.line(frame, center, y_point, (0,255,0), 2)
            #cv2.line(frame, center, x_point, (0,255,255), 2)

        contours, _ = cv2.findContours(ball_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            contour = sorted(contours, key=cv2.contourArea, reverse=True)[0].squeeze()
            cv2.drawContours(geo_frame, [contour], 0, (0,255,0), 2)

            ball_center = get_contour_centroid(contour)
            cv2.circle(frame, ball_center, 3, (0,0,255), -1, lineType=cv2.LINE_AA)
            
        
        # Display the received frame
        cv2.imshow('edge_calibration', platform_edges)
        cv2.imshow('platform_calibration', platform_masked_frame)
        cv2.imshow('ball_calibration', ball_masked_frame)
        cv2.imshow('rotation_calibration', geo_frame)
        cv2.imshow('final', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        count += 1
        if count % 20 == 0:
            fps = count/(time.time() - start)
            print(f'fps: {fps}')
            count = 0
            start = time.time()

    # Release the video window and close the sockets when done
    cv2.destroyAllWindows()
    sock.close()

    # Updating params with ending values
    params['platform']['low_threshold'] = platform_low_vals.tolist()
    params['platform']['high_threshold'] = platform_high_vals.tolist()
    params['platform']['brightness'] = platform_brightness
    params['platform']['contrast'] = platform_contrast
    params['platform']['rotation'] = platform_rotation
    params['ball']['low_threshold'] = ball_low_vals.tolist()
    params['ball']['high_threshold'] = ball_high_vals.tolist()
    params['ball']['brightness'] = ball_brightness
    params['ball']['contrast'] = ball_contrast
    save_calibration_params(params)