"""youbot_controller controller."""

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *


#--------------------------TESTING IMPORTS ONLY--CHANGE FOR SUBMISSION--------------------------
from zomb_detection import *
#--------------------------END TESTING IMPORTS----------------------------------------------

import numpy as np
#import cv2

   
#------------------CHANGE CODE BELOW HERE ONLY--------------------------
#define functions here for making decisions and using sensor inputs
    
SPEED = 14
INFINITY = float('inf')

wheels = []

#kinematics functions, translated to Python
def base_set_wheel_velocity(t, velocity):
    t.setPosition(INFINITY);
    t.setVelocity(velocity);

def base_set_wheel_speeds_helper(speeds):
    for i in range(4):
        base_set_wheel_velocity(wheels[i], speeds[i])
       
def base_reset():
  speeds = [0.0, 0.0, 0.0, 0.0];
  base_set_wheel_speeds_helper(speeds);
 
def base_forwards():
  speeds = [SPEED, SPEED, SPEED, SPEED]
  base_set_wheel_speeds_helper(speeds)

def base_backwards():
  speeds = [-SPEED, -SPEED, -SPEED, -SPEED]
  base_set_wheel_speeds_helper(speeds)

def base_turn_left():
  speeds = [-SPEED, SPEED, -SPEED, SPEED]
  base_set_wheel_speeds_helper(speeds)

def base_turn_right():
  speeds = [SPEED, -SPEED, SPEED, -SPEED]
  base_set_wheel_speeds_helper(speeds)

def base_strafe_left():
  speeds = [SPEED, -SPEED, -SPEED, SPEED]
  base_set_wheel_speeds_helper(speeds)

def base_strafe_right():
  speeds = [-SPEED, SPEED, SPEED, -SPEED]
  base_set_wheel_speeds_helper(speeds)

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
    
    
    #fr.setPosition(float('inf'))
    #fl.setPosition(float('inf'))
    #br.setPosition(float('inf'))
    #bl.setPosition(float('inf'))
    
    wheels.extend([fr, fl, br, bl])
    
    
    i = 0
    j = 0
           

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
         #move fwd some, then do 90 degree left turn

        if i < 50:
            base_forwards()
        
        if i == 50:
            base_reset() 
            base_turn_left()  
                 
        if i == 55:
            i = 0
        
        i += 1
        
        
        #camera stuff
        
        #get image as nested list
        image = camera1.getImageArray()
        
        #wrapper to C
        imageRaw = camera1.getImage()
        
        #convert nested list to 3D numpy array, ready to be input to OpenCV
        npimg = np.array(image) #shape of array should be (128, 64, 3) (128x64 pix, 3 color chan)
        
        #if img came back non-null
        if image:
            #example of how to get RGB components of each pixel
            for x in range(0, camera1.getWidth()):
                for y in range(0, camera1.getHeight()):
                    red   = image[x][y][0]
                    green = image[x][y][1]
                    blue  = image[x][y][2]
                    gray  = (red + green + blue) / 3
                    #print('r='+str(red)+' g='+str(green)+' b='+str(blue))
                    
        if j == 0:  
            #obtain a test img of scene
            camera1.saveImage("testZomb9.jpg", 90)
             
        
        # print(lidar.getRangeImage())
        """while receiver.getQueueLength() > 0:
            print(receiver.getData())
            receiver.nextPacket()"""
        
        #make decisions using inputs if you choose to do so
         
        #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
        
        
    return 0   


main()
