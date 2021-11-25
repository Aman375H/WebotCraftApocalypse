#test zombie detection, drawing from test images from img folder


import numpy as np
import cv2
import argparse
import colorsys
from imutils.convenience import *


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

LOWER_AQUA_HSV = np.array([78, 0, 0])
UPPER_AQUA_HSV = np.array([92, 255, 255])

SHOW_ALL_CONTS = False #shows not just zombie, but ALL contours found in img with threshold color
CONT_COL_GREEN = (0, 255, 0)

def process_img(img):
    #print(CV_AQUA_RGB)
    #print(colorsys.rgb_to_hsv(CV_AQUA_RGB[0], CV_AQUA_RGB[1], CV_AQUA_RGB[2]))

    aqua  = np.uint8([[[179, 230, 0]]])
    aqua = cv2.cvtColor(aqua, cv2.COLOR_BGR2HSV)
    #print(aqua)

    #open the image using OpenCV
    opened_image = cv2.imread(img)

    #print("The passed image shape is (rows, cols, channels) ", opened_image.shape)

    find_zomb_on_img(opened_image, color = None)
    #print("Processing single image...")
            
def find_zomb_on_img(image, color):
    #converts the BGR color space of image to HSV color space (supposedly better for filtering?)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #scale up images for displaying
    scale_percent = 1000 # percent of original size to scale imgs
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    dim = (width, height)
  
    # resize image
    resized_og = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)  
    resized_hsv = cv2.resize(hsv, dim, interpolation = cv2.INTER_AREA)
    
    #prepare mask to overlay
    mask = cv2.inRange(hsv, LOWER_AQUA_HSV, UPPER_AQUA_HSV)
    resized_mask = cv2.resize(mask, dim, interpolation = cv2.INTER_AREA)


    # find contours in the thresholded image
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = grab_contours(cnts)

    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:10] #sorts contours according to their area from largest to smallest.

    largestCont = cnts[0] #store the largest contour

    winning_yval = 100000
    winning_xval = 0

    #initialize x and y disp to 0
    #**NOTE: if we can't detect square in this frame, [0, 0] WILL BE RETURNED, THROWING DATA OFF
    xDisp = 0
    yDisp = 0

    num_of_conts_in_frame_that_satisy_dims = 0

    #print(len(cnts), "contours found in frame")

    # loop over the contours
    for c in cnts:
        #print("Contour found")

        #compute the "centers of mass" of each contour in the image
        M = cv2.moments(c)

        compute_com_success = True

        # make sure not div by 0
        if (M["m00"] != 0):
            cX = int((M["m10"] / M["m00"]) * scale_percent / 100)
            cY = int((M["m01"] / M["m00"]) * scale_percent / 100)
        else:
            compute_com_success = False

        # multiply the contour (x, y)-coordinates by the resize ratio,
        # then draw the contours and the name of the shape on the image
        c = c.astype("float")

        #print(c)
        #print("Length of this cntr is", c.size / 2)

        #placeholder to keep track of top left and bottom rt pts
        top_left = []
        top_left_sum = 1000000
        btm_rt = []
        btm_rt_sum = 0

        #find corresponding max y val for that x val
        for i in range(0, c.shape[0]): #iterate over all points in contour
            this_x = c[i, :, 0]
            this_y = c[i, :, 1]

            xysum = this_x + this_y

            #iterate over all points in contour checking sum of x and y vals
            if xysum > btm_rt_sum:
                btm_rt_sum = xysum
                btm_rt = np.array([this_x, this_y])

            if xysum < top_left_sum:
                top_left_sum = xysum
                top_left = np.array([this_x, this_y])
                
        #print("Top left is", top_left, "bottom rt is", btm_rt)

        #if top left or btm rt couldn't be found, jump to next contour
        if btm_rt.size == 0 or top_left.size == 0:
            #print("An extraneous rect or square was found in a frame/img, because btm_rt or top_left is []")
            continue

        #otherwise we can successfully compute pixel dimensions of the zombie
        width = btm_rt[0] - top_left[0]
        height = btm_rt[1] - top_left[1]
        dimsum = width + height


    #print(num_of_conts_in_frame_that_satisy_dims, "satisfying square contours found in frame")

    #if we specified to show all the contours, draw all
    if SHOW_ALL_CONTS:
        for c in cnts:
            c[:, :, 0] = c[:, :, 0] * scale_percent / 100
            c[:, :, 1] = c[:, :, 1] * scale_percent / 100

            c = c.astype("int")

            #draw the contour in green on original cropped color image
            cv2.drawContours(resized_og, [c], -1, CONT_COL_GREEN, 2)

    else:
        #just draw zombie contour
        largestCont[:, :, 0] = largestCont[:, :, 0] * scale_percent / 100
        largestCont[:, :, 1] = largestCont[:, :, 1] * scale_percent / 100

        largestCont = largestCont.astype("int")

        #draw the contour in green on original cropped color image
        cv2.drawContours(resized_og, [largestCont], -1, CONT_COL_GREEN, 2)


    cv2.imshow("Processed image", resized_mask)

    # show the output image
    cv2.imshow("Original image", resized_og)
    cv2.imshow("HSV image", resized_hsv)

    #wait indefinitely for a key press
    k = cv2.waitKey(0)

    #if key is q, close all windows and quit program
    if k == ord('q'): 
        cv2.destroyAllWindows()
        quit()

#Main entrance point
if __name__ == '__main__':
    # construct the argument parse and parse the arguments
    ap = argparse.ArgumentParser()

    # arg to input a single image or video
    ap.add_argument("-i", "--image", required=False,
                    help="path to the input image")
    ap.add_argument("-v", "--video", required=False,
                    help="path to the input video")
    ap.add_argument("-s", "--step", required=False, help="whether to manually step through frames")
    args = vars(ap.parse_args())

    if args["image"] != None and args["video"] != None:
        print("You can't pass in an image and a video at the same time.")
    elif args["image"] != None:
        process_img(args["image"])

    #if a video is being passed    
    elif args["video"] != None:
        if args["step"] == None:
            #set globs
            cam = cv2.VideoCapture(args["video"])

            #execute the application.
            #Start Qt event loop unless running in interactive mode or using pyside.
            if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
                QtGui.QApplication.instance().exec_()

        else:
            #process the video    
            process_video(args["video"])

    else:
        try:
            print("Please pass img as arg")
        except KeyboardInterrupt:
            print('Interrupted')
            cv2.destroyAllWindows()
            sys.exit(0)
