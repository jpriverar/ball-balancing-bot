import numpy as np
import cv2
from calibration_receiver import load_calibration_params, draw_polar_line, get_contour_centroid

class ImageProcessor:
    def __init__(self):
        self.params = load_calibration_params()


    def preprocess_frame(self, frame):
        blurred_frame = cv2.GaussianBlur(frame, (5,5), 0)
        lab_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2LAB)
        return lab_frame

    
    def find_platform(self, frame):
        mask = cv2.inRange(frame, self.params['color_theshold']['platform']['low'], self.params['color_theshold']['platform']['high'])
        blurred = cv2.GaussianBlur(mask, (5,5), 0)
        edges = cv2.Canny(blurred, self.params['edge_detection']['min'], self.params['edge_detection']['max'])
        
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            contour = sorted(contours, key=cv2.contourArea, reverse=True)[0].squeeze()
            hull = cv2.convexHull(contour)

            # Drawing the convex hull contour to extract lines
            hull_frame = np.zeros_like(mask)
            cv2.drawContours(hull_frame, [hull], 0, 255, 3)
            center = get_contour_centroid(hull)
        return center


    def find_ball(self, frame):
        mask = cv2.inRange(frame, self.params['color_theshold']['ball']['low'], self.params['color_theshold']['ball']['high'])
        blurred = cv2.GaussianBlur(mask, (5,5), 0)
        edges = cv2.Canny(blurred, self.params['edge_detection']['min'], self.params['edge_detection']['max'])

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            contour = sorted(contours, key=cv2.contourArea, reverse=True)[0].squeeze()

            center = get_contour_centroid(contour)
        return center