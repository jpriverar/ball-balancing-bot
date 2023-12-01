import numpy as np
import cv2
from utils import *

class ImageProcessor:
    def __init__(self, platform_params, ball_params):
        self.platform_params = platform_params
        self.ball_params = ball_params


    def preprocess_platform_frame(self, frame):
        frame = np.clip(frame + self.platform_params['brightness'], 0, 255)
        frame = np.uint8(np.clip(frame * self.platform_params['contrast'], 0, 255))
        blurred_frame = cv2.GaussianBlur(frame, (5,5), 0)
        lab_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2LAB)
        return lab_frame
    

    def preprocess_ball_frame(self, frame):
        frame = np.clip(frame + self.ball_params['brightness'], 0, 255)
        frame = np.uint8(np.clip(frame * self.ball_params['contrast'], 0, 255))
        blurred_frame = cv2.GaussianBlur(frame, (5,5), 0)
        lab_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2LAB)
        return lab_frame

    
    def find_platform(self, frame):
        mask = cv2.inRange(frame, self.platform_params['low_platform'], self.platform_params['high_platform'])
        blurred = cv2.GaussianBlur(mask, (5,5), 0)
        edges = cv2.Canny(blurred, 150, 255)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            contour = sorted(contours, key=cv2.contourArea, reverse=True)[0].squeeze()
            hull = cv2.convexHull(contour)
            center = get_contour_centroid(hull)
        return center


    def find_ball(self, frame):
        mask = cv2.inRange(frame, self.ball_params['low_platform'], self.ball_params['high_platform'])
        blurred = cv2.GaussianBlur(mask, (5,5), 0)
        edges = cv2.Canny(blurred, 150, 255)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            contour = sorted(contours, key=cv2.contourArea, reverse=True)[0].squeeze()
            center = get_contour_centroid(contour)
        return center