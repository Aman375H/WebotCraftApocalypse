"""youbot_controller controller."""

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *

import numpy as np
import cv2
   
#------------------CHANGE CODE BELOW HERE ONLY--------------------------
#define functions here for making decisions and using sensor inputs
    
############### CAMERA CODE ###################

#Zombies: 1.82m tall


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
LOWER_BLUE_HSV = np.array([95, 150, 50])
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
LOWER_WHITE_HSV = np.array([0, 0, 0]) #100, 0, 0
UPPER_WHITE_HSV = np.array([180, 40, 100]) #115, 70, 100
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


SHOW_ALL_CONTS = True #if TRUE, shows ALL contours found in img with threshold colors
CONT_COL_GREEN = (0, 255, 0) #contour line color (for testing)


#element class: an element of interest in the image
class Element:
    def __init__(self, name, com, cntr_pts, width, height, center):
        self.name = name
        self.com = com
        self.cntr_pts = cntr_pts
        self.width = width
        self.height = height
        self.center = center


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
        x, y, w, h = cv2.boundingRect(contour)


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
        
        center = [x + (w/2), y + (h/2)]

        elements.append(Element(name, com, contour, w, h, center))
        #cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)


def locate_element(element_name, hsv_img, elements):
    # print("Now attempting to locate", element_name)

    element_color_filters = filter_bank[element_name]
    #print(element_color_filters)

    #scan for both reds and both wall colors
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
    
    
################# ROBOT CODE #####################
            
# movement globals
SPEED = 14
INFINITY = float('inf')
IMG_X_CENTER = 64
LAST_BERRY_CONSUMED = ""
time_stump_seen = -50

wheels = []

# arm globals
ARM_FRONT_FLOOR=0
ARM_FRONT_PLATE=1
ARM_FRONT_CARDBOARD_BOX=2
ARM_RESET=3
ARM_BACK_PLATE_HIGH=4
ARM_BACK_PLATE_LOW=5
ARM_HANOI_PREPARE=6
ARM_BACK_LEFT=7
ARM_LEFT=8
ARM_FRONT_LEFT=9
ARM_FRONT=10
ARM_FRONT_RIGHT=11
ARM_RIGHT=12
ARM_BACK_RIGHT=13
ARM1=0
ARM2=1
ARM3=2
ARM4=3
ARM5=4

arm_elements = [None, None, None, None, None]

current_height = ARM_RESET
current_orientation = ARM_FRONT

def arm_init(robot):
    arm_elements[ARM1] = robot.getDevice("arm1")
    arm_elements[ARM2] = robot.getDevice("arm2")
    arm_elements[ARM3] = robot.getDevice("arm3")
    arm_elements[ARM4] = robot.getDevice("arm4")
    arm_elements[ARM5] = robot.getDevice("arm5")

    arm_elements[ARM2].setVelocity(0.5);

def arm_reset():
    arm_elements[ARM1].setPosition(0.0)
    arm_elements[ARM2].setPosition(1.57)
    arm_elements[ARM3].setPosition(-2.635)
    arm_elements[ARM4].setPosition(1.78)
    arm_elements[ARM5].setPosition(0.0)
    
def arm_set_berry_pos():
    arm_elements[ARM2].setPosition(-1.13)
    arm_elements[ARM3].setPosition(-0.65)
    arm_elements[ARM4].setPosition(0.0)
    arm_elements[ARM5].setPosition(0.0)

# gripper functions
LEFT = 0
RIGHT = 1

MIN_POS = 0.0
MAX_POS = 0.025
OFFSET_WHEN_LOCKED = 0.021

fingers = [None, None]

def gripper_init(robot):
  fingers[LEFT] = robot.getDevice("finger1")
  fingers[RIGHT] = robot.getDevice("finger2")

  fingers[LEFT].setVelocity(0.03)
  fingers[RIGHT].setVelocity(0.03)
  fingers[LEFT].setPosition(MAX_POS)
  fingers[RIGHT].setPosition(MAX_POS)
  

#kinematics functions, translated to Python
def base_set_wheel_velocity(t, velocity):
    t.setPosition(INFINITY)
    t.setVelocity(velocity)

def base_set_wheel_speeds_helper(speeds):
    for i in range(4):
        base_set_wheel_velocity(wheels[i], speeds[i])
       
def base_reset():
    speeds = [0.0, 0.0, 0.0, 0.0]
    base_set_wheel_speeds_helper(speeds)
 
def base_forwards():
    speeds = [SPEED, SPEED, SPEED, SPEED]
    base_set_wheel_speeds_helper(speeds)

def base_backwards():
    speeds = [-SPEED, -SPEED, -SPEED, -SPEED]
    base_set_wheel_speeds_helper(speeds)

def base_turn_left():
    speeds = [SPEED, -SPEED, SPEED, -SPEED]
    base_set_wheel_speeds_helper(speeds)

def base_turn_right():
    speeds = [-SPEED, SPEED, -SPEED, SPEED]
    base_set_wheel_speeds_helper(speeds)

def base_strafe_left():
    speeds = [SPEED, -SPEED, -SPEED, SPEED]
    base_set_wheel_speeds_helper(speeds)

def base_strafe_right():
    speeds = [-SPEED, SPEED, SPEED, -SPEED]
    base_set_wheel_speeds_helper(speeds)
    
def base_tilt_left(tilt_factor):
    speeds = [SPEED, SPEED*(1-tilt_factor), SPEED, SPEED*(1-tilt_factor)]
    base_set_wheel_speeds_helper(speeds)
    
def base_tilt_right(tilt_factor):
    speeds = [SPEED*(1-tilt_factor), SPEED, SPEED*(1-tilt_factor), SPEED]
    base_set_wheel_speeds_helper(speeds)


##### higher level behaviors as finite state machines #####

# wander behavior when nothing interesting is spotted
class wander():
    def __init__(self):
        self.state = 0
        self.turn_state = 0
        self.prev_time = 0
        self.prev_turn_time = 0
        self.wall_threshold = 0
    def output(self, i, walls, health, energy, l_avg_depth, r_avg_depth):
        # try turning other way periodically
        if i - self.prev_turn_time > 350:
            print("change wander direction")
            self.turn_state = 1 - self.turn_state #switch turn "state" to its complement
            self.prev_turn_time = i #save current simulation clock time
            
        # if nothing is in view, wander in circles and along walls, searching for elements of interest
        if len(walls) == 0:
            if self.state == 0 and i - self.prev_time >= 20 or self.state == 1 and i-self.prev_time >= 60:
                self.state = 1 - self.state #it's time to switch wandering state
                self.prev_time = i
            
            # survey
           
            #if in state 0, just turn in place based on turn_state 
            elif self.state == 0:
                if self.turn_state:
                    base_turn_left() 
                else:
                    base_turn_right()
           
            #if in state 1, march forwards
            elif self.state == 1:
                base_forwards()
            return
        
        if l_avg_depth - r_avg_depth > 1:
            base_turn_left()
            return
        elif r_avg_depth - l_avg_depth > 1:
            base_turn_right()
            return
            
        #if have some supposed wall contours in view, find largest by area    
        sorted_walls = sorted(walls, key=lambda x: x.width * x.height, reverse=True)
        target_wall = sorted_walls[0]
        
        #compute total area of all wall contours seen, combined
        total_wall_area = 0
        for wall in sorted_walls:
            total_wall_area += wall.width * wall.height
        
        # print(total_wall_area, target_wall.width*target_wall.height, target_wall.width, target_wall.height)
      
        #if still lots of wall, turn around based on current turn_state
        if total_wall_area > self.wall_threshold:
            if self.turn_state:
                base_turn_left()
            else:
                base_turn_right()
        #if not much wall area, keep moving forwards along wall to help with exploration of world
        else:
            base_forwards()

# seek berries when they are in view, else this behavior does nothing
class seek_berries():
    def __init__(self):
        self.k_p = 0.5
        self.k_d = 0.2 #PD controller params
        
        self.old_error = 0 #keep track of previous error for D control feedback element
        
    # input list of berries should only include the good ones
    def output(self, i, berries):
        if len(berries) == 0:
            return 0 #send digital low in subsumption wire
       
        # should sort berries based on which is closest to robot
        sorted_berries = sorted(berries, key=lambda x: x.width * x.height, reverse=True)
        target_berry = sorted_berries[0] #target the closest berry
        
        if target_berry.name == "orng_berr" and target_berry.width*target_berry.height <= 10:
            return 0
        
        print(target_berry.name, target_berry.width*target_berry.height)
        error = target_berry.com[0] - IMG_X_CENTER #x dist of berry from img center
        steer_cmd = (self.k_p * error + self.k_d * (error-self.old_error)) / 64 #get PD controller output
        
        self.old_error = error #save current error for D control feedback element

        #steer towards berry
        if steer_cmd > 0:
            base_tilt_right(steer_cmd)
        elif steer_cmd < 0:
            base_tilt_left(-steer_cmd)
        else:
            base_forwards() #berry already perfectly centered, move fwd
        
        return 1 #send digital high in subsumption wire
        
class avoid_bad_berries():
    def __init__(self):
        self.state = 0
        self.berry_size_threshold = 75
        self.berry_pos_threshold = 50 #how close should zombie be to ctr of image for us to consider it threat
        self.berry_xpos_min = IMG_X_CENTER - self.berry_pos_threshold
        self.berry_xpos_max = IMG_X_CENTER + self.berry_pos_threshold
        
        
    def output(self, i, berries, wanderer):
        if len(berries) == 0:
            return 0 #send digital low in subsumption wire

        #if see some zombies, find "closest" one (covering most pixel area)
        largest_berry = sorted(berries, key=lambda x: x.width * x.height, reverse=True)[0]
        largest_berry_area = largest_berry.width * largest_berry.height
        x_center = largest_berry.center[0]
        
        #only worry if zombie is in range
        if largest_berry_area < self.berry_size_threshold or x_center < self.berry_xpos_min or x_center > self.berry_xpos_max:
            #print("Zomb detected but out of rg")
            return 0 #send digital low in subsumption wire
            
        #turn away from zombie
        if IMG_X_CENTER >= x_center > self.berry_xpos_min:
            base_turn_right() 
            wanderer.turn_state = 0
            
        elif IMG_X_CENTER < x_center < self.berry_xpos_max:
            base_turn_left()
            wanderer.turn_state = 1
        
        wanderer.prev_turn_time = i
        return 1
        
        
# avoid zombies if they are in view, else this behavior does nothing
class avoid_zombies():
    def __init__(self):
        self.state = 0
        self.zomb_size_threshold = 1000
        self.zomb_pos_threshold = 58 #how close should zombie be to ctr of image for us to consider it threat
        self.zomb_xpos_min = IMG_X_CENTER - self.zomb_pos_threshold
        self.zomb_xpos_max = IMG_X_CENTER + self.zomb_pos_threshold
        
        
    def output(self, i, zombies, wanderer):
        if len(zombies) == 0:
            return 0 #send digital low in subsumption wire
            
        #if see some zombies, find "closest" one (covering most pixel area)
        largest_zombie = sorted(zombies, key=lambda x: x.width * x.height, reverse=True)[0]
        largest_zombie_area = largest_zombie.width * largest_zombie.height
        x_center = largest_zombie.center[0]
        
        #only worry if zombie is in range
        if largest_zombie_area < self.zomb_size_threshold or x_center < self.zomb_xpos_min or x_center > self.zomb_xpos_max:
            #print("Zomb detected but out of rg")
            return 0 #send digital low in subsumption wire
            
        #turn away from zombie
        if IMG_X_CENTER >= x_center > self.zomb_xpos_min:
            base_turn_right() 
            wanderer.turn_state = 0
            
        elif IMG_X_CENTER < x_center < self.zomb_xpos_max:
            base_turn_left()
            wanderer.turn_state = 1
        
        wanderer.prev_turn_time = i
        return 1

##### end of higher level behaviors #####


# dictionary of 4 complex higher level behaviors
# the other 2 behaviors were not complex enough to be in their own function
behaviors = {
    "wander": wander(),
    "seek berries": seek_berries(),
    "avoid bad berries": avoid_bad_berries(),
    "avoid zombies": avoid_zombies(),
}

# set of good berries
# update this list as berries are obtained and effects are known
# if berry is not on this list, it is bad
good_berries = set(["orng", "red", "yellow", "pink"])


#------------------CHANGE CODE ABOVE HERE ONLY--------------------------

def main():
    robot = Supervisor()

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    
    #health, energy, armour in that order 
    robot_info = [100,100,0]
    passive_wait(0.1, robot, timestep)
    pc = 0
    timer = 0
    
    robot_node = robot.getFromDef("Youbot")
    trans_field = robot_node.getField("translation")
    
    get_all_berry_pos(robot)
    
    robot_not_dead = 1
    
    #------------------CHANGE CODE BELOW HERE ONLY--------------------------
    
    #COMMENT OUT ALL SENSORS THAT ARE NOT USED. READ SPEC SHEET FOR MORE DETAILS
    """
    accelerometer = robot.getDevice("accelerometer")
    accelerometer.enable(timestep)
    
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    """
    compass = robot.getDevice("compass")
    compass.enable(timestep)
    
    camera1 = robot.getDevice("ForwardLowResBigFov")
    camera1.enable(timestep)
    
    """
    camera2 = robot.getDevice("ForwardHighResSmallFov")
    camera2.enable(timestep)
    
    camera3 = robot.getDevice("ForwardHighRes")
    camera3.enable(timestep)
    
    camera4 = robot.getDevice("ForwardHighResSmall")
    camera4.enable(timestep)
    
    camera5 = robot.getDevice("BackLowRes")
    camera5.enable(timestep)
    
    camera6 = robot.getDevice("RightLowRes")
    camera6.enable(timestep)
    
    camera7 = robot.getDevice("LeftLowRes")
    camera7.enable(timestep)
    
    camera8 = robot.getDevice("BackHighRes")
    camera8.enable(timestep)
    
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    
    lightSensor = robot.getDevice("light sensor")
    lightSensor.enable(timestep)
    
    receiver = robot.getDevice("receiver")
    receiver.enable(timestep)
    
    """
    
    rangeFinder = robot.getDevice("range-finder")
    rangeFinder.enable(timestep)
    
    """
    lidar = robot.getDevice("lidar")
    lidar.enable(timestep)"""
    
    #get the four robot wheel motors
    fr = robot.getDevice("wheel1")
    fl = robot.getDevice("wheel2")
    br = robot.getDevice("wheel3")
    bl = robot.getDevice("wheel4")
    
    
    # fr.setPosition(float('inf'))
    # fl.setPosition(float('inf'))
    # br.setPosition(float('inf'))
    # bl.setPosition(float('inf'))
    
    wheels.extend([fr, fl, br, bl])
    arm_init(robot)
    arm_set_berry_pos() #set arm to appropriate pos for berry knocking
    gripper_init(robot)
    
    global LAST_BERRY_CONSUMED
    global time_stump_seen
    
    i = 0
    j = 0
    
    #initialize previous_stats variable to hold initial stats
    previous_stats = robot_info 
    previous_direction = 0
    time_since_direction_change = 0
           

    #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
    
    
    while(robot_not_dead == 1):
        
        if(robot_info[0] < 0):
           
            robot_not_dead = 0
            print("ROBOT IS OUT OF HEALTH")
            #if(zombieTest):
            #    print("TEST PASSED")
            #else:
            #    print("TEST FAILED")
            #robot.simulationQuit(20)
            #exit()
            
        if(timer%2==0):
            trans = trans_field.getSFVec3f()
            robot_info = check_berry_collision(robot_info, trans[0], trans[2], robot)
            robot_info = check_zombie_collision(robot_info, trans[0], trans[2], robot)
            
        if(timer%16==0):
            robot_info = update_robot(robot_info)
            timer = 0
        
        if(robot.step(timestep)==-1):
            exit()
            
            
        timer += 1
        
     #------------------CHANGE CODE BELOW HERE ONLY--------------------------   
        #called every timestep
         
        i += 0.5
        
        # get camera image
        imageRaw = camera1.getImage()
        image = np.frombuffer(imageRaw, np.uint8).reshape((camera1.getHeight(), camera1.getWidth(), 4))
        
        # range image depth detection
        # each pixel is repped by dist (m) from camera sensor plane
        range_image = rangeFinder.getRangeImageArray()

        left_avg_depth = 0
        right_avg_depth = 0
        
        #compute avg depth (meters) in a 10x10 sample of middle of range image
        for j in range(32):
            for k in range(25, 35):
                left_avg_depth += range_image[j][k]
        for j in range(32, 64):
            for k in range(25, 35):
                right_avg_depth += range_image[j][k]
        left_avg_depth /= 320
        right_avg_depth /= 320
        avg_depth = (left_avg_depth + right_avg_depth) / 2
        
        # movement check to avoid getting stuck
        direction = compass.getValues()[1]
        
        #if haven't changed direction in a while, probably stuck on something
        if 10 < i - time_stump_seen < 25 or 60 < time_since_direction_change < 75:
            print("evasive maneuvers")
            
            #start moving backwards, continue for 15 cc
            base_backwards()
            time_since_direction_change += 1
            continue #go to next while loop iteration, ignore all below
            
        #if robot doesn't seem to be changing direction rn, add some time to the stuck stopwatch
        if abs(direction - previous_direction) < 0.001:
            time_since_direction_change += 0.5
        else:
            time_since_direction_change = 0
            
        #store current direction to compare on next loop iteration    
        previous_direction = direction
        
        
        # find all elements of interest in image
        elements = find_elements_on_img(image)
        
        berries = []
        bad_berries = []
        zombies = []
        walls = []
        stumps = []
        
        for element in elements:
            if element.name[-4:] == "zomb":
                zombies.append(element)
            elif element.name[-4:] == "berr":
                if element.name[:-5] not in good_berries:
                    bad_berries.append(element) #if the berry is bad, treat it like a zombie
                else:
                    berries.append(element) 
            elif element.name == "wall" and avg_depth < 3.8:
                walls.append(element) #see a wall and are really close to something (avoid mistaking grey mtns for walls)
            elif element.name == "stump":
                stumps.append(element)
                
        # print("zombies:", len(zombies), "berries:", len(berries), "walls:", len(walls), "stumps:", len(stumps))
        
        #if detected some berries, find "closest" one 
        if len(berries):
            largest_berry = sorted(berries, key=lambda x: x.width * x.height, reverse=True)[0]
            if largest_berry.width * largest_berry.height > 4000: #assume robot consumed berry at this pt
                LAST_BERRY_CONSUMED = largest_berry.name[:-5] #keep track of berry just consumed
                
        # choose behaviors according to subsumption architecture
        if behaviors['avoid bad berries'].output(i,bad_berries, behaviors["wander"]):
            #BERRY AVOIDANCE HATH SUBSUMED
            print("avoiding bad berry")
        
        elif robot_info[2] == 0 and behaviors['avoid zombies'].output(i, zombies, behaviors["wander"]):
            #ZOMBIE AVOIDANCE HATH SUBSUMED
            print("avoiding zombies")
            
        #seek berries only if health or energy are less than 90
        elif (robot_info[0] < 90 or robot_info[1] < 90) and behaviors['seek berries'].output(i, berries):
            #BERRY SEEKING HATH SUBSUMED
            #print("seeking berries")
            
            #if this berry removed energy, mark it as bad by removing from goodlist
            if previous_stats[1] > robot_info[1] + 15 and LAST_BERRY_CONSUMED in good_berries:
                print("--------------------------------------------")
                print("bad berry", LAST_BERRY_CONSUMED, previous_stats, robot_info)
                good_berries.remove(LAST_BERRY_CONSUMED)
        elif len(stumps) > 0 and stumps[0].height * stumps[0].width > 2000:
            #STUMP CHARGING HATH SUBSUMED
            print("charging stump")
            base_forwards()
            time_stump_seen = i
        else:
            # print("wandering")
            #DEFAULT WANDERING BEHAVIOR, GOT NOTHING BETTER TO DO
            behaviors['wander'].output(i, walls, robot_info[0], robot_info[1], left_avg_depth, right_avg_depth)
        
        
        previous_stats = robot_info.copy() #some memory for saving and comparing curr robot stats to previous
         
        #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
        
        
    return 0   


main()
