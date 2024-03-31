import numpy as np
import cv2
from utils import *

class ImageProcessor:
    def __init__(self, platform_params, ball_params):
        self.platform_params = platform_params
        self.ball_params = ball_params

        # To have a low pass filter of the positions
        self.prev_platform_center = (0,0)
        self.prev_ball_center = (0,0)


    def preprocess_platform_frame(self, frame):
        frame = np.uint8(np.clip(np.int32(frame) + self.platform_params['brightness'], 0, 255))
        frame = np.uint8(np.clip(np.int32(frame) * self.platform_params['contrast'], 0, 255))
        blurred_frame = cv2.GaussianBlur(frame, (5,5), 0)
        lab_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2LAB)
        return lab_frame
    

    def preprocess_ball_frame(self, frame):
        frame = np.uint8(np.clip(np.int32(frame) + self.ball_params['brightness'], 0, 255))
        frame = np.uint8(np.clip(np.int32(frame) * self.ball_params['contrast'], 0, 255))
        blurred_frame = cv2.GaussianBlur(frame, (5,5), 0)
        lab_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2LAB)
        return lab_frame

    
    def find_platform(self, frame):
        mask = cv2.inRange(frame, np.array(self.platform_params['low_threshold'], dtype=np.uint8), np.array(self.platform_params['high_threshold'], dtype=np.uint8))
        blurred = cv2.GaussianBlur(mask, (5,5), 0)
        edges = cv2.Canny(blurred, 150, 255)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return None
        
        contour = sorted(contours, key=cv2.contourArea, reverse=True)[0].squeeze()
        hull = cv2.convexHull(contour)
        center = get_contour_centroid(hull)
        filtered_center = (int(center[0] * self.platform_params['alpha'] +  (1 - self.platform_params['alpha']) * self.prev_platform_center[0]),
                           int(center[1] * self.platform_params['alpha'] +  (1 - self.platform_params['alpha']) * self.prev_platform_center[1]))
        self.prev_platform_center = filtered_center
        return filtered_center
    

    def get_fixed_platform_center(self):
        return tuple(self.platform_params['fixed_center'])


    def find_ball(self, frame):
        mask = cv2.inRange(frame, np.array(self.ball_params['low_threshold'], dtype=np.uint8), np.array(self.ball_params['high_threshold'], dtype=np.uint8))
        blurred = cv2.GaussianBlur(mask, (5,5), 0)

        edges = cv2.Canny(blurred, 150, 255)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return None

        contour = sorted(contours, key=cv2.contourArea, reverse=True)[0].squeeze()
        center = get_contour_centroid(contour)
        filtered_center = (int(center[0] * self.ball_params['alpha'] +  (1 - self.ball_params['alpha']) * self.prev_ball_center[0]),
                           int(center[1] * self.ball_params['alpha'] +  (1 - self.ball_params['alpha']) * self.prev_ball_center[1]))
        self.prev_ball_center = filtered_center
        return filtered_center