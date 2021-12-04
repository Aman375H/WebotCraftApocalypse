"""youbot_controller controller."""

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *


#--------------------------TESTING IMPORTS ONLY--CHANGE FOR SUBMISSION--------------------------
from zomb_detection import *
from zombiemasktest import *
#--------------------------END TESTING IMPORTS----------------------------------------------

import numpy as np
import cv2

#------------------CHANGE CODE BELOW HERE ONLY--------------------------
#define functions here for making decisions and using sensor inputs


# movement globals
SPEED = 14
INFINITY = float('inf')
IMG_X_CENTER = 64
LAST_BERRY_CONSUMED = ""

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
    def output(self, i, walls, health, energy):
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
            if self.state == 0:
                if self.turn_state:
                    base_turn_left() 
                else:
                    base_turn_right()
           
            #if in state 1, march forwards
            elif self.state == 1:
                base_forwards()
            return
            
        #if have some supposed wall contours in view, find largest by area    
        sorted_walls = sorted(walls, key=lambda x: x.width * x.height, reverse=True)
        target_wall = sorted_walls[0]
        
        #compute total area of all wall contours seen, combined
        total_wall_area = 0
        for wall in sorted_walls:
            total_wall_area += wall.width * wall.height
        
        # print(total_wall_area, target_wall.width*target_wall.height, target_wall.width, target_wall.height)
        
        #if lots of wall, turn around based on current turn_state
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
        if 0 > x_center > self.zomb_xpos_min or x_center == 0:
            base_turn_right() 
            wanderer.turn_state = 0
            
        else:
            base_turn_left()
            wanderer.turn_state = 1
        
        wanderer.prev_turn_time = i
        return 1

##### end of higher level behaviors #####


# dictionary of higher level behaviors
behaviors = {
    "wander": wander(),
    "seek berries": seek_berries(),
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
    robot_info = [100, 100, 0]
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
    # arm_set_height(ARM_HANOI_PREPARE)
    arm_set_berry_pos() #set arm to appropriate pos for berry knocking
    gripper_init(robot)
    
    global LAST_BERRY_CONSUMED
    
    i = 0
    j = 0
    
    #initialize previou_stats variable to hold initial stats
    previous_stats = robot_info 
    previous_direction = 0
    time_since_direction_change = 0
           

    #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
    
    
    while(robot_not_dead == 1):
        #check if health is empty
        if(robot_info[0] < 0):
            #if so, she dead
            robot_not_dead = 0
            print("ROBOT IS OUT OF HEALTH")
            
            #if(zombieTest):
            #    print("TEST PASSED")
            #else:
            #    print("TEST FAILED")
            #robot.simulationQuit(20)
            #exit()
            
        #check for berry and zombie presence, not every cc   
        if(timer % 2 == 0):
            trans = trans_field.getSFVec3f()
            robot_info = check_berry_collision(robot_info, trans[0], trans[2], robot)
            robot_info = check_zombie_collision(robot_info, trans[0], trans[2], robot)
            
        #every 2 sec        
        if(timer % 16 == 0):
            #update the 3 robot stats, and print them
            robot_info = update_robot(robot_info)
            timer = 0 #reset clock
        
        if(robot.step(timestep) == -1):
            exit()
            
        #increment clock
        timer += 1
        
     #------------------CHANGE CODE BELOW HERE ONLY--------------------------   
        #called every timestep
         
        i += 0.5
        
        
        #camera stuff
        
        #wrapper to C
        imageRaw = camera1.getImage()
        image = np.frombuffer(imageRaw, np.uint8).reshape((camera1.getHeight(), camera1.getWidth(), 4))
        #camera1.saveImage("C:/Users/aman3/OneDrive/Documents\\GitHub\\WebotCraftApocalypse\\testing\\img\\orng.jpg", 90)
        #camera1.saveImage("/home/nodog/WebotCraftApocalypse/testing/img/stumpTest.jpg", 90)
        
        
        # range image depth detection
        # each pixel is repped by dist (m) from camera sensor plane
        range_image = rangeFinder.getRangeImageArray()
        #print("Range img dims are", np.shape(range_image))

        avg_depth = 0
        
        #compute avg depth (meters) in a 10x10 sample of middle of range image
        for j in range(27, 37):
            for k in range(25, 35):
                avg_depth += range_image[j][k]
        avg_depth /= 100
        
        
        # movement check to avoid getting stuck
        direction = compass.getValues()[1]
        
        #if haven't changed direction in a while, probably stuck on sth
        if 60 < time_since_direction_change < 75:
            #print("evasive maneuvers")
            
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
        zombies = []
        walls = []
        stumps = []
        
        for element in elements:
            if element.name[-4:] == "zomb":
                zombies.append(element)
            elif element.name[-4:] == "berr":
                if element.name[:-5] not in good_berries:
                    zombies.append(element) #if the berry is bad, treat it like a zombie
                else:
                    berries.append(element) 
            elif element.name == "wall" and avg_depth < 3.7:
                walls.append(element) #see a wall and are really close to something (avoid mistaking grey mtns for walls)
            elif element.name == "stump":
                stumps.append(element)
        
        #if detected some berries, find "closest" one 
        if len(berries):
            largest_berry = sorted(berries, key=lambda x: x.width * x.height, reverse=True)[0]
            if largest_berry.width * largest_berry.height > 4000: #assume robot consumed berry at this pt
                LAST_BERRY_CONSUMED = largest_berry.name[:-5] #keep track of berry just consumed
        #print("last berry:", LAST_BERRY_CONSUMED)
        
        
            

        # choose behaviors according to subsumption architecture
        if behaviors['avoid zombies'].output(i, zombies, behaviors["wander"]):
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
            #print("charging stump")
            base_forwards()
        else:
            #print("wandering")
            #DEFAULT WANDERING BEHAVIOR, GOT NOTHING BETTER TO DO
            behaviors['wander'].output(i, walls, robot_info[0], robot_info[1])
        
        
        previous_stats = robot_info.copy() #some memory for saving and comparing curr robot stats to previous
        
        #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
        
        
    return 0


main()
