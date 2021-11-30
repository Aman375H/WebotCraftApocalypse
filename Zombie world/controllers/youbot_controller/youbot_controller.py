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
current_berry = ""

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

    arm_set_height(ARM_RESET);
    arm_set_orientation(ARM_FRONT);

def arm_reset():
    arm_elements[ARM1].setPosition(0.0)
    arm_elements[ARM2].setPosition(1.57)
    arm_elements[ARM3].setPosition(-2.635)
    arm_elements[ARM4].setPosition(1.78)
    arm_elements[ARM5].setPosition(0.0)

def arm_set_height(height):
    if height == ARM_FRONT_FLOOR:
        arm_elements[ARM2].setPosition(-0.97)
        arm_elements[ARM3].setPosition(-1.55)
        arm_elements[ARM4].setPosition(-0.61)
        arm_elements[ARM5].setPosition(0.0)
    elif height == ARM_FRONT_PLATE:
        arm_elements[ARM2].setPosition(-0.62)
        arm_elements[ARM3].setPosition(-0.98)
        arm_elements[ARM4].setPosition(-1.53)
        arm_elements[ARM5].setPosition(0.0)
    elif height == ARM_FRONT_CARDBOARD_BOX:
        arm_elements[ARM2].setPosition(0.0)
        arm_elements[ARM3].setPosition(-0.77)
        arm_elements[ARM4].setPosition(-1.21)
        arm_elements[ARM5].setPosition(0.0)
    elif height == ARM_RESET:
        arm_elements[ARM2].setPosition(1.57)
        arm_elements[ARM3].setPosition(-2.635)
        arm_elements[ARM4].setPosition(1.78)
        arm_elements[ARM5].setPosition(0.0)
    elif height == ARM_BACK_PLATE_HIGH:
        arm_elements[ARM2].setPosition(0.678)
        arm_elements[ARM3].setPosition(0.682)
        arm_elements[ARM4].setPosition(1.74)
        arm_elements[ARM5].setPosition(0.0)
    elif height == ARM_BACK_PLATE_LOW:
        arm_elements[ARM2].setPosition(0.92)
        arm_elements[ARM3].setPosition(0.42)
        arm_elements[ARM4].setPosition(1.78)
        arm_elements[ARM5].setPosition(0.0)
    elif height == ARM_HANOI_PREPARE:
        arm_elements[ARM2].setPosition(-0.4)
        arm_elements[ARM3].setPosition(-1.2)
        arm_elements[ARM4].setPosition(-np.pi/2)
        arm_elements[ARM5].setPosition(np.pi/2)
    else:
        print("arm_height() called with a wrong argument")
        return

    current_height = height

def arm_set_orientation(orientation):
    if orientation == ARM_BACK_LEFT:
        arm_elements[ARM1].setPosition(-2.949)
    elif orientation == ARM_LEFT:
        arm_elements[ARM1].setPosition(-np.pi/2)
    elif orientation == ARM_FRONT_LEFT:
        arm_elements[ARM1].setPosition(-0.2)
    elif orientation == ARM_FRONT:
        arm_elements[ARM1].setPosition(0.0)
    elif orientation == ARM_FRONT_RIGHT:
        arm_elements[ARM1].setPosition(0.2)
    elif orientation == ARM_RIGHT:
        arm_elements[ARM1].setPosition(np.pi/2)
    elif orientation == ARM_BACK_RIGHT:
        arm_elements[ARM1].setPosition(2.949)
    else:
      print("arm_set_side() called with a wrong argument\n")
      return
    current_orientation = orientation

def arm_set_sub_arm_rotation(arm, radian):
    arm_elements[arm].setPosition(radian)


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
        self.wall_threshold = 1000
    def output(self, i, walls):
        # if nothing is in view, wander in circles, searching
        if len(walls) == 0:
            if i % 16 == 0:
                self.state = 1 - self.state
            # survey
            if self.state == 0:
                base_turn_right()
            # march forwards
            elif self.state == 1:
                base_forwards()
            return
            
        sorted_walls = sorted(berries, key=lambda x: x.width*x.height, reverse=True)
        target_wall = sorted_walls[0]
        
        if target_wall.width*target_wall.height > THRESHOLD:
            base_turn_right
        else:
            base_forwards()

# seek berries when they are in view, else this behavior
# does nothing
class seek_berries():
    def __init__(self):
        self.k_p = 0.5
        self.k_d = 0.2
        
        self.old_error = 0
        
    # input list of berries should only include the good ones
    def output(self, i, berries):
        global current_berry
        
        if len(berries) == 0:
            return 0
        
        # should sort berries based on which is closest to robot
        sorted_berries = sorted(berries, key=lambda x: x.width*x.height, reverse=True)
        target_berry = sorted_berries[0]
        
        error = target_berry.com[0] - IMG_X_CENTER
        steer_cmd = (self.k_p*error + self.k_d*(error-self.old_error))/64
        
        self.old_error = error

        if steer_cmd > 0:
            base_tilt_right(steer_cmd)
        elif steer_cmd < 0:
            base_tilt_left(-steer_cmd)
        else:
            base_forwards()
        
        current_berry = target_berry.name[:-5]
        
        return 1 
        
# avoid zombies if they are in view, else this behavior
# does nothing
class avoid_zombies():
    def __init__(self):
        self.state = 0
    # TODO
    def output(self, i, zombies):
        if len(zombies) == 0:
            return 0

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
good_berries = set(["ornge", "red", "yellow", "pink"])

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
    """accelerometer = robot.getDevice("accelerometer")
    accelerometer.enable(timestep)
    
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    
    compass = robot.getDevice("compass")
    compass.enable(timestep)"""
    
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
    
    rangeFinder = robot.getDevice("range-finder")
    rangeFinder.enable(timestep)
    
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
    arm_set_height(ARM_HANOI_PREPARE)
    
    i = 0
    j = 0
    previous_stats = robot_info
    previous_berry = current_berry
           

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

        elements = find_elements_on_img(image)
        
        berries = []
        zombies = []
        walls = []
        stumps = []
        
        for element in elements:
            if element.name[-4:] == "zomb":
                zombies.append(element)
            elif element.name[-4:] == "berr" and element.com[1] > 27:
                if element.name[:-5] not in good_berries:
                    zombies.append(element)
                else:
                    berries.append(element)
            elif element.name == "wall":
                walls.append(element)
            elif element.name == "stump":
                stumps.append(element)    
                    
        #make decisions using inputs if you choose to do so
        
        if behaviors['avoid zombies'].output(i, []):
            continue
        elif behaviors['seek berries'].output(i, berries):
            if previous_stats[1] > robot_info[1] + 15:
                print("--------------------------------------------")
                print("bad berry", previous_berry, previous_stats, robot_info)
                good_berries.remove(previous_berry)
                print(good_berries)
            continue
        else:
            behaviors['wander'].output(i, walls)
         
        previous_stats = robot_info.copy()
        previous_berry = current_berry[:]
        
        #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
        
        
    return 0


main()
