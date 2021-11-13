#test zombie detection, drawing from test images from img folder


import numpy as np
import cv2
import argparse
import colorsys


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

def process_img(img):
    print(CV_AQUA_RGB)
    print(colorsys.rgb_to_hsv(CV_AQUA_RGB[0], CV_AQUA_RGB[1], CV_AQUA_RGB[2]))

    aqua  = np.uint8([[[179, 230, 0]]])
    aqua = cv2.cvtColor(aqua, cv2.COLOR_BGR2HSV)
    print(aqua)

    #open the image using OpenCV
    opened_image = cv2.imread(img)

    print("The passed image shape is (rows, cols, channels) ", opened_image.shape)

    find_zomb_on_img(opened_image, color = None)
    print("Processing single image...")
            
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
            start_realtime()
        except KeyboardInterrupt:
            print('Interrupted')
            cv2.destroyAllWindows()
            sys.exit(0)
