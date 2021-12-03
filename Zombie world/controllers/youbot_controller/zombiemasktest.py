#test zombie detection, drawing from test images from img folder

#Zombies: 1.82m tall

import numpy as np
from numpy.core.defchararray import find
import cv2
# import argparse
# import colorsys
# from imutils.convenience import *

#ZOMBS
WEBOTS_AQUA = np.array([0, 0.9, 0.7])
WEBOTS_BLUE = np.array([0, 0.5, 1])
WEBOTS_GREEN = np.array([0, 0.7, 0])
WEBOTS_PURPLE = np.array([0.6, 0.2, 1])

CV_AQUA_RGB = np.multiply(WEBOTS_AQUA, 255)
CV_BLUE_RGB = np.multiply(WEBOTS_BLUE, 255)
CV_GREEN_RGB = np.multiply(WEBOTS_GREEN, 255)
CV_PURPLE_RGB = np.multiply(WEBOTS_PURPLE, 255)

#aqua zombies
LOWER_AQUA_HSV = np.array([78, 150, 50])
UPPER_AQUA_HSV = np.array([92, 255, 255])
AQUA_ZOMB_COLOR = [LOWER_AQUA_HSV, UPPER_AQUA_HSV]

#blue zombies
LOWER_BLUE_HSV = np.array([95, 200, 100])
UPPER_BLUE_HSV = np.array([115, 255, 255])
BLUE_ZOMB_COLOR = [LOWER_BLUE_HSV, UPPER_BLUE_HSV]

#purple zombies
LOWER_PURPLE_HSV = np.array([120, 150, 50])
UPPER_PURPLE_HSV = np.array([145, 255, 255])
PURPLE_ZOMB_COLOR = [LOWER_PURPLE_HSV, UPPER_PURPLE_HSV]

#green zombies
LOWER_GREEN_HSV = np.array([50, 100, 0])
UPPER_GREEN_HSV = np.array([70, 255, 255])
GREEN_ZOMB_COLOR = [LOWER_GREEN_HSV, UPPER_GREEN_HSV]


#BERRIES
WEBOTS_YELLOW = np.array([1, 0.9, 0])
WEBOTS_ORANGE = np.array([0.9, 0.5, 0.3])
WEBOTS_PINK = np.array([0.9, 0.5, 0.7])
WEBOTS_RED = np.array([1, 0.2, 0.1])

CV_YELLOW_RGB = np.multiply(WEBOTS_YELLOW, 255)
CV_ORANGE_RGB = np.multiply(WEBOTS_ORANGE, 255)
CV_PINK_RGB = np.multiply(WEBOTS_PINK, 255)
CV_RED_RGB = np.multiply(WEBOTS_RED, 255)

#yellow berries
LOWER_YELLOW_HSV = np.array([24, 150, 0])
UPPER_YELLOW_HSV = np.array([36, 255, 255])
YELLOW_BERR_COLOR = [LOWER_YELLOW_HSV, UPPER_YELLOW_HSV]

#orange berries
LOWER_ORANGE_HSV = np.array([7, 120, 0])
UPPER_ORANGE_HSV = np.array([21, 255, 255])
ORANGE_BERR_COLOR = [LOWER_ORANGE_HSV, UPPER_ORANGE_HSV]

#pink berries
LOWER_PINK_HSV = np.array([140, 30, 140])
UPPER_PINK_HSV = np.array([175, 180, 200])
PINK_BERR_COLOR = [LOWER_PINK_HSV, UPPER_PINK_HSV]

#red berries
LOWER_RED_HSV = np.array([0, 150, 0])
UPPER_RED_HSV = np.array([8, 255, 255])
RED_BERR_COLOR = [LOWER_RED_HSV, UPPER_RED_HSV]

#red wraps around to higher end of hue spectrum
LOWER_RED2_HSV = np.array([173, 90, 180])
UPPER_RED2_HSV = np.array([190, 255, 255])
RED2_BERR_COLOR = [LOWER_RED2_HSV, UPPER_RED2_HSV]

#WALLS
WEBOTS_WHITE = np.array([1, 1, 1])

CV_WHITE_RGB = np.multiply(WEBOTS_WHITE, 255) 

#shaded_wall
LOWER_WHITE_HSV = np.array([100, 0, 0])
UPPER_WHITE_HSV = np.array([115, 70, 100])
SHADED_WALL_COLOR = [LOWER_WHITE_HSV, UPPER_WHITE_HSV]

#sunny wall
LOWER_WHITE2_HSV = np.array([0, 0, 150])
UPPER_WHITE2_HSV = np.array([255, 20, 255])
SUNNY_WALL_COLOR = [LOWER_WHITE2_HSV, UPPER_WHITE2_HSV]

#stumps
LOWER_BLACK_HSV = np.array([0, 0, 0])
UPPER_BLACK_HSV = np.array([180, 255, 30])
STUMP_COLOR = [LOWER_BLACK_HSV, UPPER_BLACK_HSV]

#filter bank
filter_bank = {"aqua_zomb": AQUA_ZOMB_COLOR, "blue_zomb": BLUE_ZOMB_COLOR, "purp_zomb": PURPLE_ZOMB_COLOR, "green_zomb": GREEN_ZOMB_COLOR, 
"yellow_berr": YELLOW_BERR_COLOR, "orng_berr": ORANGE_BERR_COLOR, "pink_berr": PINK_BERR_COLOR, "red_berr": [RED_BERR_COLOR, RED2_BERR_COLOR], 
"wall" : [SHADED_WALL_COLOR, SUNNY_WALL_COLOR], "stump": STUMP_COLOR}

""" IGNORE
CV_AQUA_HSV = np.array([167, 100, 90])
CV_BLUE_HSV = np.array([210, 100, 100])
CV_GREEN_HSV = np.array([120, 100, 70])
CV_PURPLE_HSV = np.array([270, 80, 100])"""

SHOW_ALL_CONTS = True #if TRUE, shows ALL contours found in img with threshold colors
CONT_COL_GREEN = (0, 255, 0) #contour line color (for testing)


#element class: an element of interest in the image
class Element:
    def __init__(self, name, com, cntr_pts, width, height):
        self.name = name
        self.com = com
        self.cntr_pts = cntr_pts
        self.width = width
        self.height = height

    #TODO


def find_and_save_contours(mask_img, name, elements):
    #find contours in the thresholded image
    cnts = cv2.findContours(mask_img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0]

    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:10] #sorts contours according to their area from largest to smallest.

    #make sure some pixels in imaged matched filters
    if (len(cnts) == 0):
        # print("No contours found in img for element", name)
        return

    largestCont = cnts[0] #store the largest contour

    #print(len(cnts), "contours found in frame")
    largestCont = largestCont.astype("float")

    #placeholder to keep track of top left and bottom rt pts
    top_left = []
    top_left_sum = 1000000
    btm_rt = []
    btm_rt_sum = 0


    for contour in cnts:
        #find corresponding max y val for that x val
        x,y,w,h = cv2.boundingRect(contour)
        
        # for i in range(0, contour.shape[0]): #iterate over all points in contour
            # this_x = contour[i, :, 0]
            # this_y = contour[i, :, 1]

            # xysum = this_x + this_y #sum x and y vals of pt

            # #checking sum of x and y vals
            # if xysum > btm_rt_sum: #find bottom rt pt
                # btm_rt_sum = xysum
                # btm_rt = np.array([this_x, this_y])

            # if xysum < top_left_sum: #find top left pt
                # top_left_sum = xysum
                # top_left = np.array([this_x, this_y])
                
        #print("Top left is", top_left, "bottom rt is", btm_rt)

        #compute approximate pixel dimensions of the zombie
        # width = btm_rt[0] - top_left[0]
        # height = btm_rt[1] - top_left[1]

        # compute the "centers of mass" of each contour in the image
        M = cv2.moments(contour)

        compute_com_success = True

        # make sure not div by 0
        if (M["m00"] != 0):
            cX = int((M["m10"] / M["m00"]))
            cY = int((M["m01"] / M["m00"]))
        else:
            compute_com_success = False
            cX = int(contour[0, :, 0])
            cY = int(contour[0, :, 1])

        com = [cX, cY]

        elements.append(Element(name, com, contour, w, h))
        #cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)


def locate_element(element_name, hsv_img, elements):
    # print("Now attempting to locate", element_name)

    element_color_filters = filter_bank[element_name]
    #print(element_color_filters)

    #scan for both reds
    if element_name == "red_berr" or element_name == "wall":
        for filter_pair in element_color_filters:
            mask = cv2.inRange(hsv_img, filter_pair[0], filter_pair[1])

            find_and_save_contours(mask, element_name, elements)

    else:
        mask = cv2.inRange(hsv_img, element_color_filters[0], element_color_filters[1])
        find_and_save_contours(mask, element_name, elements)


def find_elements_on_img(image):
    elements = []
    
    #converts the BGR color space of image to HSV color space (supposedly better for filtering?)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #scale up images for displaying
    scale_percent = 1000 # percent of original size to scale imgs
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    dim = (width, height)
  
    #resize image
    resized_og = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)  
    resized_hsv = cv2.resize(hsv, dim, interpolation = cv2.INTER_AREA)
    
    for key in filter_bank:
        locate_element(key, hsv, elements)

    #draw out all elements
    for element in elements:
        #scale and draw this element's contour
        element.cntr_pts[:, :, 0] = element.cntr_pts[:, :, 0] * scale_percent / 100
        element.cntr_pts[:, :, 1] = element.cntr_pts[:, :, 1] * scale_percent / 100
        cv2.drawContours(resized_og, [element.cntr_pts], -1, CONT_COL_GREEN, 2)

        if element.com != None:
            # print(element.name, "found centered at", element.com)
            cv2.putText(resized_og, element.name, (element.com[0] * int(scale_percent / 100), element.com[1] * int(scale_percent / 100)), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 2)
            
    return elements
    #resized_mask = cv2.resize(mask, dim, interpolation = cv2.INTER_AREA)

    #print("Zombie pixel width is ", width[0].astype("int"), ", pixel ht is ", height[0].astype("int"))

    #dimsum = width + height

    #print(num_of_conts_in_frame_that_satisy_dims, "satisfying square contours found in frame")

    #draw contours
    #multiply the contour (x, y)-coordinates by the resize ratio,
    #then draw the contours and the name of the shape on the image
    #largestCont[:, :, 0] = largestCont[:, :, 0] * scale_percent / 100
    #largestCont[:, :, 1] = largestCont[:, :, 1] * scale_percent / 100

    #largestCont = largestCont.astype("int")

    #draw the contour in green on original cropped color image
    #cv2.drawContours(resized_og, [largestCont], -1, CONT_COL_GREEN, 2)

    """
    #if we specified to show all the contours, draw all rest of cnts
    if SHOW_ALL_CONTS:
        for c in cnts[1:len(cnts)]:
            c[:, :, 0] = c[:, :, 0] * scale_percent / 100
            c[:, :, 1] = c[:, :, 1] * scale_percent / 100

            c = c.astype("int")

            #draw the contour in green on original cropped color image
            cv2.drawContours(resized_og, [c], -1, CONT_COL_GREEN, 2)"""

    #display HSV, binary, and original img with contours drawn
    #cv2.imshow("HSV image - Q to quit", resized_hsv)
    #cv2.imshow("Processed image - Q to quit", resized_mask)
    # cv2.imshow("Original image - Q to quit", resized_og)
    
    # #wait indefinitely for a key press
    # k = cv2.waitKey(0)

    # #if key is q, close all windows and quit program
    # if k == ord('q'): 
        # cv2.destroyAllWindows()
        # quit()


def process_img(img):
    #test and print out colors
    #print(CV_RED_RGB)
    #print(colorsys.rgb_to_hsv(CV_AQUA_RGB[0], CV_AQUA_RGB[1], CV_AQUA_RGB[2]))
    #org = np.uint8([[[26, 51, 255]]])
    #org = cv2.cvtColor(org, cv2.COLOR_BGR2HSV)
    #print(org)

    #open the image using OpenCV
    opened_image = cv2.imread(img)

    #print("The passed image shape is (rows, cols, channels) ", opened_image.shape)


    return find_elements_on_img(opened_image)

    #print("Processing single image...")
            

    
#Main entrance point
if __name__ == '__main__':
    # construct the argument parse and parse the arguments
    ap = argparse.ArgumentParser()

    # arg to input a single image or video
    ap.add_argument("-i", "--image", required=False, help="path to the input image")
    args = vars(ap.parse_args())

    if args["image"] != None:
        process_img(args["image"])
    else:
        try:
            print("Please pass img as arg")
        except KeyboardInterrupt:
            print('Interrupted')
            cv2.destroyAllWindows()
            quit()