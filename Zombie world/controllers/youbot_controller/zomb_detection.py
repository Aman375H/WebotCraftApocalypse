import numpy as np
import cv2


WEBOTS_AQUA = np.array([0, 0.9, 0.7])
WEBOTS_BLUE = np.array([0, 0.5, 1])
WEBOTS_GREEN = np.array([0, 0.7, 0])
WEBOTS_PURPLE = np.array([0.6, 0.2, 1])

CV_AQUA_RGB = np.multiply(WEBOTS_AQUA, 255)
CV_BLUE_RGB = np.multiply(WEBOTS_AQUA, 255)
CV_GREEN_RGB = np.multiply(WEBOTS_AQUA, 255)
CV_PURPLE_RGB = np.multiply(WEBOTS_AQUA, 255)

CV_AQUA_HSV = np.array([167, 100, 90])
CV_BLUE_HSV = np.array([210, 100, 100])
CV_GREEN_HSV = np.array([120, 100, 70])
CV_PURPLE_HSV = np.array([270, 80, 100])

LOWER_AQUA_HSV = np.array([157, 84, 76])
UPPER_AQUA_HSV = np.array([180, 100, 100])

class ShapeDetector:
    #TBD
            
def find_zomb_on_img(image, color):
    #converts the BGR color space of image to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    #prepare mask to overlay
    mask = cv2.inRange(hsv, LOWER_AQUA_HSV, UPPER_AQUA_HSV)
    cv2.imshow("Processed image", mask)
    
    #Testing in progress outside of webots
