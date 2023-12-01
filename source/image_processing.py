import numpy as np
import cv2
from utils import *

class ImageProcessor:
    def __init__(self):
        self.params = load_calibration_params()


    def preprocess_platform_frame(self, frame):
        frame = np.clip(frame + self.params['platform']['brightness'], 0, 255)
        frame = np.uint8(np.clip(frame * self.params['platform']['contrast'], 0, 255))
        blurred_frame = cv2.GaussianBlur(frame, (5,5), 0)
        lab_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2LAB)
        return lab_frame
    

    def preprocess_ball_frame(self, frame):
        frame = np.clip(frame + self.params['ball']['brightness'], 0, 255)
        frame = np.uint8(np.clip(frame * self.params['ball']['contrast'], 0, 255))
        blurred_frame = cv2.GaussianBlur(frame, (5,5), 0)
        lab_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2LAB)
        return lab_frame

    
    def find_platform(self, frame):
        mask = cv2.inRange(frame, self.params['platform']['low_platform'], self.params['platform']['high_platform'])
        blurred = cv2.GaussianBlur(mask, (5,5), 0)
        edges = cv2.Canny(blurred, self.params['edge_detection']['min'], self.params['edge_detection']['max'])
        
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            contour = sorted(contours, key=cv2.contourArea, reverse=True)[0].squeeze()
            hull = cv2.convexHull(contour)
            center = get_contour_centroid(hull)
        return center


    def find_ball(self, frame):
        mask = cv2.inRange(frame, self.params['ball']['low_platform'], self.params['ball']['high_platform'])
        blurred = cv2.GaussianBlur(mask, (5,5), 0)
        edges = cv2.Canny(blurred, self.params['edge_detection']['min'], self.params['edge_detection']['max'])

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            contour = sorted(contours, key=cv2.contourArea, reverse=True)[0].squeeze()
            center = get_contour_centroid(contour)
        return center