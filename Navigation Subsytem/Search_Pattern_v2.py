from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
from dronekit_sitl import SITL
import time
import argparse
import numpy as np

import pyrealsense2 as rs
import cv2
import tensorflow as tf

parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect', default='127.0.0.1:14551')
args = parser.parse_args()
connection_string = args.connect
# Connect to the Vehicle
print ('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(args.connect, baud=921600, wait_ready=True)
#921600 is the baudrate that you have set in the mission plannar or qgc
'''This code has a hardcoded logo location in and the goal of this code is to get the drone to locate the logo using a search pattern.
In this code the functions are called and it operates like this:
SearchPatternleft()
SearchPatternStraight()
SearchPatternRight()
This sequence of code is done in a loop until the x area of the logo is found.
Once the x area of the logo is found it then calls the Logo_y_search() function which hones in on the y coordinate area of the logo
Once the y area is found it then begins the homing sequene by calling the homing_x() function. This functions adjust the drone to the correct x coordinate of the logo.
Then it calls the homing_y() function that hones in on the exact y coordinate of the logo
Once the y coordinate is found it then lands
'''
def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Check that vehicle has reached takeoff altitude
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
class Movement:#Class that defines all the movement directions
    def __init__(self):
        self.velocity_x = 0
        self.velocity_y = 0
        self.velocity_z = 0
        self.duration   = 0
        self.loc = [0,0] #Array to input the current location of the drone
        self.new_location = 0 #This is used to check the amount of distance traveled on the x-axis
        self.side_boundary_location = 0 #This is used to check amout of distance traveled in the y-axis

    def arm_and_takeoff(self,aTargetAltitude):
            print("Basic pre-arm checks")
            # Don't let the user try to arm until autopilot is ready
            while not vehicle.is_armable:
                print(" Waiting for vehicle to initialise...")
                time.sleep(1)

            print("Arming motors")
            # Copter should arm in GUIDED mode
            vehicle.mode = VehicleMode("GUIDED")
            vehicle.armed = True

            while not vehicle.armed:
                print(" Waiting for arming...")
                time.sleep(1)

            print("Taking off!")
            vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

            # Check that vehicle has reached takeoff altitude
            while True:
                print(" Altitude: ", vehicle.location.global_relative_frame.alt)
                starting_x_loc = vehicle.location.local_frame.north
                starting_y_loc = vehicle.location.local_frame.east
                self.loc = [starting_x_loc, starting_y_loc]  # Test Code
                print("Current location is", self.loc)
                # Break and return from function just below target altitude.
                if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                    print("Reached target altitude")
                    break
                time.sleep(1)
#Function get the starting location of the drone
    def starting_loc_function(self):
        global starting_x_location  # To allow the x location variable is used in all functions
        global starting_y_location  # To allow the y location variable to be used in all functions
        starting_x_location = int(vehicle.location.local_frame.north)
        starting_y = int(vehicle.location.local_frame.east)
        starting_y_location = starting_y
    def upwards_velocity(self,velocity_z_input,duration_input): #Defines what it means to move upwards
        self.velocity_x = 0
        self.velocity_y = 0
        self.velocity_z = velocity_z_input*-1
        self.duration = duration_input
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
            0b0000111111000111,  # type_mask
            0, 0, 0,  # x, y, z positions (not used)
            self.velocity_x, self.velocity_y, self.velocity_z,  # m/s
            0, 0, 0,  # x, y, z acceleration
            0, 0)
        for x in range(0, duration_input):
            print("Flying Upwards")
            starting_x_loc = int(vehicle.location.local_frame.north)
            starting_y_loc = int(vehicle.location.local_frame.east)
            self.loc = [starting_x_loc, starting_y_loc]
            print("Altitude is ", vehicle.location.global_relative_frame.alt)
            vehicle.send_mavlink(msg)
            time.sleep(1)
    def downwards_velocity(self,velocity_neg_z_input,duration_input): #Defines what it means to move downwards
        self.velocity_x = 0
        self.velocity_y = 0
        self.velocity_z = velocity_neg_z_input
        self.duration = duration_input
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
            0b0000111111000111,  # type_mask
            0, 0, 0,  # x, y, z positions (not used)
            self.velocity_x, self.velocity_y, self.velocity_z,  # m/s
            0, 0, 0,  # x, y, z acceleration
            0, 0)
        for x in range(0, duration_input):
            print("Flying Downwards")
            starting_x_loc = int(vehicle.location.local_frame.north)
            starting_y_loc = int(vehicle.location.local_frame.east)
            self.loc = [starting_x_loc, starting_y_loc]
            print("Altitude is ", vehicle.location.global_relative_frame.alt)
            vehicle.send_mavlink(msg)
            time.sleep(1)
    def forward_velocity(self, velocity_x_input,duration_input):
        self.velocity_x = velocity_x_input
        self.velocity_y = 0
        self.velocity_z = 0
        self.duration = duration_input
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
            0b0000111111000111,  # type_mask
            0, 0, 0,  # x, y, z positions (not used)
            self.velocity_x, self.velocity_y, self.velocity_z,  # m/s
            0, 0, 0,  # x, y, z acceleration
            0, 0)
        for x in range(0, duration_input):
            print("Flying forwards")
            starting_x_loc = int(vehicle.location.local_frame.north)
            starting_y_loc = int(vehicle.location.local_frame.east)
            self.loc = [starting_x_loc, starting_y_loc]
            self.new_location = self.loc[0] - starting_x_location
            self.side_boundary_location = self.loc[1] - starting_y_location
            print("Current location is", self.loc)
            vehicle.send_mavlink(msg)
            time.sleep(1)
    def backwards_velocity(self, velocity_negative_x_input,duration_input):
        self.velocity_x = velocity_negative_x_input*-1
        self.velocity_y = 0
        self.velocity_z = 0
        self.duration = duration_input
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
            0b0000111111000111,  # type_mask
            0, 0, 0,  # x, y, z positions (not used)
            self.velocity_x, self.velocity_y, self.velocity_z,  # m/s
            0, 0, 0,  # x, y, z acceleration
            0, 0)
        for x in range(0, duration_input):
            print("Flying backwards")
            starting_x_loc = int(vehicle.location.local_frame.north)
            starting_y_loc = int(vehicle.location.local_frame.east)
            self.loc = [starting_x_loc, starting_y_loc]
            print("Current location is", self.loc)
            vehicle.send_mavlink(msg)
            time.sleep(1)
    def right_velocity(self, velocity_right_input,duration_input):
        self.velocity_x = 0
        self.velocity_y = velocity_right_input
        self.velocity_z = 0
        self.duration = duration_input
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
            0b0000111111000111,  # type_mask
            0, 0, 0,  # x, y, z positions (not used)
            self.velocity_x, self.velocity_y, self.velocity_z,  # m/s
            0, 0, 0,  # x, y, z acceleration
            0, 0)
        for x in range(0, duration_input):
            print("Flying right")
            starting_x_loc = int(vehicle.location.local_frame.north)
            starting_y_loc = int(vehicle.location.local_frame.east)
            self.loc = [starting_x_loc, starting_y_loc]
            self.new_location = self.loc[0] - starting_x_location
            self.side_boundary_location = self.loc[1] - starting_y_location
            print("Current location is", self.loc)
            vehicle.send_mavlink(msg)
            time.sleep(1)
    def left_velocity(self, velocity_negative_left_input,duration_input):
        self.velocity_x = 0
        self.velocity_y = velocity_negative_left_input*-1
        self.velocity_z = 0
        self.duration = duration_input
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
            0b0000111111000111,  # type_mask
            0, 0, 0,  # x, y, z positions (not used)
            self.velocity_x, self.velocity_y, self.velocity_z,  # m/s
            0, 0, 0,  # x, y, z acceleration
            0, 0)
        for x in range(0, duration_input):
            print("Flying left")
            starting_x_loc = int(vehicle.location.local_frame.north)
            starting_y_loc = int(vehicle.location.local_frame.east)
            self.loc = [starting_x_loc, starting_y_loc]
            self.new_location = self.loc[0] - starting_x_location
            self.side_boundary_location = self.loc[1] - starting_y_location
            print("Current location is", self.loc)
            vehicle.send_mavlink(msg)
            time.sleep(1)
    def right_diagonol_velocity(self, velocity_diagonol_input,duration_input):
        self.velocity_x = velocity_diagonol_input
        self.velocity_y = velocity_diagonol_input
        self.velocity_z = 0
        self.duration = duration_input
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
            0b0000111111000111,  # type_mask
            0, 0, 0,  # x, y, z positions (not used)
            self.velocity_x, self.velocity_y, self.velocity_z,  # m/s
            0, 0, 0,  # x, y, z acceleration
            0, 0)
        for x in range(0, duration_input):
            print("Flying right diagonal")
            vehicle.send_mavlink(msg)
            time.sleep(1)
    def left_diagonol_velocity(self, velocity_diagonol_input,duration_input):
        self.velocity_x = velocity_diagonol_input*-1
        self.velocity_y = velocity_diagonol_input
        self.velocity_z = 0
        self.duration = duration_input
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
            0b0000111111000111,  # type_mask
            0, 0, 0,  # x, y, z positions (not used)
            self.velocity_x, self.velocity_y, self.velocity_z,  # m/s
            0, 0, 0,  # x, y, z acceleration
            0, 0)
        for x in range(0, duration_input):
            print("Flying left diagonal")
            vehicle.send_mavlink(msg)
            time.sleep(1)
    def right_neg_diagonol_velocity(self, velocity_diagonol_input,duration_input):
        self.velocity_x = velocity_diagonol_input
        self.velocity_y = velocity_diagonol_input*-1
        self.velocity_z = 0
        self.duration = duration_input
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
            0b0000111111000111,  # type_mask
            0, 0, 0,  # x, y, z positions (not used)
            self.velocity_x, self.velocity_y, self.velocity_z,  # m/s
            0, 0, 0,  # x, y, z acceleration
            0, 0)
        for x in range(0, duration_input):
            print("Flying negative right diagonal")
            vehicle.send_mavlink(msg)
            time.sleep(1)
    def left_neg_diagonol_velocity(self, velocity_diagonol_input,duration_input):
        self.velocity_x = velocity_diagonol_input*-1
        self.velocity_y = velocity_diagonol_input*-1
        self.velocity_z = 0
        self.duration = duration_input
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
            0b0000111111000111,  # type_mask
            0, 0, 0,  # x, y, z positions (not used)
            self.velocity_x, self.velocity_y, self.velocity_z,  # m/s
            0, 0, 0,  # x, y, z acceleration
            0, 0)
        for x in range(0, duration_input):
            print("Flying negative left diagonal")
            vehicle.send_mavlink(msg)
            time.sleep(1)

#Functions for the search pattern and homing sequences
def Logo_y_search(): #This is the function to to get near the y logo location area
    if p1.loc[1] <15:
        while True:
            if p1.loc[1] >13:
                print("Nearby Logo y Location")
                break
            else:
                p1.right_velocity(1,1) #Testing Value should be 1
    elif p1.loc[1]>15:
        while True:
            if p1.loc[1] >12 and p1.loc[1] <15:
                print("Nearby Logo y Location")
                break
            else:
                p1.left_velocity(1,1) #Testing Value should be 1
    else:
        while True:
            if p1.loc[1] >13:
                print("Nearby Logo y Location")
                break
            else:
                p1.right_velocity(1,1)#Testing Value should be 1
print("Found x and y area of logo!")
def Search_pattern_straight(): #This is defining how the drone moves straight in its search pattern technique
    if p1.loc[0]>76:
        print("Nearby Logo x location")
        Logo_y_search()
    else:
        p1.forward_velocity(15,1)
        if p1.loc[0]>76:
            print("Nearby Logo x location")
            print("Now to start homing in on the y logo position")
            Logo_y_search()
        else:
            p1.forward_velocity(15, 1)
def Search_pattern_left(): #This is defining the boundaries and how the drone moves left in the search pattern technique
    current_y_location_ = p1.loc[1] #This defines the starting location of the left sequence
    print("Starting y in this sequence yo",current_y_location_)
    while True:
            current_y_location_1 = p1.loc[1] - current_y_location_ #This checks the distance traveled
            print("Search pattern left boundary number is",int(current_y_location_1))
            if current_y_location_1 >48 or current_y_location_1<-48:
                print("Drone met 50 yard boundary time to go forward")
                break
            elif p1.loc[0]>76:
                print("Nearby Logo x location")
                break
            else:
                p1.left_velocity(2, 1) #Testing value should be 2
    if p1.loc[0]>76:
        print("Time to start homing in on the logo y position")
        Logo_y_search()
    else:
        Search_pattern_straight()
def Search_pattern_right(): #This is defining the boundaries and how the drone moves right in the search patttern technique
    current_y_location_=p1.loc[1]
    while True:
        current_y_location_1=p1.loc[1]-current_y_location_
        print("Search pattern right boundary number is:",int(current_y_location_1))
        if current_y_location_1 >48 or current_y_location_1<-48:
                print("Drone met 50 yard boundary time to go forward")
                break
        elif p1.loc[0]>76:
                print("Nearby Logo x location")
                break
        else:
                p1.right_velocity(2, 1) #Testing value should be 2
    if p1.loc[0] > 76:
        print("Time to start homing in on the y position")
        Logo_y_search()
    else:
        Search_pattern_straight()

def condition_yaw(heading, relative=False): #This is set the drone straight in the beginning of the code
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
def homing_sequence_y(): #This is to home in on the exact coordinates of the logo
    condition_yaw(0)
    if p1.loc[1] > 15:
        while True:
            if p1.loc[1] == 14 or p1.loc[1] ==15:
                print("Y coordinate found")
                break
            else:
                p1.left_velocity(1,1) #Testing value should be 1
        print("Logo has been identified")
        print("Current location is", p1.loc)
        print("Logo Location is:", logo)
        print("Time to land")
        vehicle.mode = VehicleMode("LAND")
        vehicle.close()
    elif p1.loc[1]<15:
        while True:
            if p1.loc[1] == 14 or p1.loc[1] == 15:
                print("Y coordinate found")
                break
            else:
                p1.right_velocity(1,1) #Testing Value
        print("Logo has been identified")
        print("Current location is", p1.loc)
        print("Logo Location is:", logo)
        print("Time to land")
        vehicle.mode = VehicleMode("LAND")
        vehicle.close()
    else:
        print("Y coordinate found")
        print("Logo has been identified")
        print("Current location is",p1.loc)
        print("Logo Location is:",logo)
        print("Time to land")
        vehicle.mode = VehicleMode("LAND")
        vehicle.close()
def homing_sequence_x(): #This is to hone in on the exact coordinates of the logo
    if p1.loc[0] > 80:
        while True:
            if p1.loc[0] ==79 or p1.loc[0] ==80:
                print("X coordinate found")
                break
            else:
                p1.backwards_velocity(1,1) #Testing value should be 1
        homing_sequence_y()
    elif p1.loc[0]<80:
        while True:
            if p1.loc[0] == 79 or p1.loc[0] == 80:
                print("X coordinate found")
                break
            else:
                p1.forward_velocity(1,1) #Testing value should be 1m/s instead
        homing_sequence_y()
    else:
        print("X coordinate found")
        homing_sequence_y()

#Stanley's functions
def evaluate_image(image_array, dim, model):
    tamu = tf.keras.models.load_model(model)
    image_array = cv2.resize(image_array, (dim, dim))
    ims = (np.expand_dims(image_array, 0))
    pred = tamu.predict(ims)

    if (pred[0][1] < 0.9):
        return False #logo is not identified
    else:
        return True #logo is identified

    return False

def crop_nine_ways(image_array, original_dim, model):
    tamu = tf.keras.models.load_model(model)
    image_array = cv2.resize(image_array, (original_dim, original_dim))
    height = image_array.shape[0]
    width = image_array.shape[1]
    
    #1-1
    left = 0
    top = 0
    right = width / 3
    bottom = height / 3
    im1 = image_array[top:bottom, left:right]
    #plt.imshow(im1)
    #plt.show()
    im1 = (np.expand_dims(im1, 0))
    pred1 = tamu.predict(im1)
    print("Section 1-1")
    print(pred1)
    
    #1-2
    left = width / 3
    top = 0
    right = 2 * width / 3
    bottom = height / 3
    im2 = image_array[top:bottom, left:right]
    #plt.imshow(im2)
    #plt.show()
    im2 = (np.expand_dims(im2, 0))
    pred2 = tamu.predict(im2)
    print("Section 1-2")
    print(pred2)
    
    #1-3
    left = 2 * width / 3
    top = 0
    right = width
    bottom = height / 3
    im3 = image_array[top:bottom, left:right]
    #plt.imshow(im3)
    #plt.show()
    im3 = (np.expand_dims(im3, 0))
    pred3 = tamu.predict(im3)
    print("Section 1-3")
    print(pred3)
    
    #2-1
    left = 0
    top = height / 3
    right = width / 3
    bottom = 2 * height / 3
    im4 = image_array[top:bottom, left:right]
    #plt.imshow(im4)
    #plt.show()
    im4 = (np.expand_dims(im4, 0))
    pred4 = tamu.predict(im4)
    print("Section 2-1")
    print(pred4)
    
    #2-2
    left = width / 3
    top = height / 3
    right = 2 * width / 3
    bottom = 2 * height / 3
    im5 = image_array[top:bottom, left:right]
    #plt.imshow(im5)
    #plt.show()
    im5 = (np.expand_dims(im5, 0))
    pred5 = tamu.predict(im5)
    print("Section 2-2")
    print(pred5)
    
    #2-3
    left = 2 * width / 3
    top = height / 3
    right = width
    bottom = 2 * height / 3
    im6 = image_array[top:bottom, left:right]
    #plt.imshow(im6)
    #plt.show()
    im6 = (np.expand_dims(im6, 0))
    pred6 = tamu.predict(im6)
    print("Section 2-3")
    print(pred6)
    
    #3-1
    left = 0
    top = 2 * height / 3
    right = width / 3
    bottom = height
    im7 = image_array[top:bottom, left:right]
    #plt.imshow(im7)
    #plt.show()
    im7 = (np.expand_dims(im7, 0))
    pred7 = tamu.predict(im7)
    print("Section 3-1")
    print(pred7)
    
    #3-2
    left = width / 3
    top = 2 * height / 3
    right = 2 * width / 3
    bottom = height
    im8 = image_array[top:bottom, left:right]
    #plt.imshow(im8)
    #plt.show()
    im8 = (np.expand_dims(im8, 0))
    pred8 = tamu.predict(im8)
    print("Section 3-2")
    print(pred8)
    
    #3-3
    left = 2 * width / 3
    top = 2 * height / 3
    right = width
    bottom = height
    im9 = image_array[top:bottom, left:right]
    #plt.imshow(im9)
    #plt.show()
    im9 = (np.expand_dims(im9, 0))
    pred9 = tamu.predict(im9)
    print("Section 3-3")
    print(pred9)
    
    #calculate region with the highest probablity of having the logo
    maximum = max(pred1[0][1], pred2[0][1], pred3[0][1], pred4[0][1], pred5[0][1], pred6[0][1], pred7[0][1], pred8[0][1], pred9[0][1])
    
    if(pred1[0][1] == maximum):
        return [1, 1]
    elif(pred2[0][1] == maximum):
        return [1, 2]
    elif(pred3[0][1] == maximum):
        return [1, 3]
    elif(pred4[0][1] == maximum):
        return [2, 1]
    elif(pred5[0][1] == maximum):
        return [2, 2]
    elif(pred6[0][1] == maximum):
        return [2, 3]
    elif(pred7[0][1] == maximum):
        return [3, 1]
    elif(pred8[0][1] == maximum):
        return [3, 2]
    elif(pred9[0][1] == maximum):
        return [3, 3]

def shift_drone_position(image_array, original_dim, model, elevation):
    mat = crop_nine_ways(image_array, original_dim, model)
    
    # The RGB FOV is 69 deg x 42 deg (H X V)
    # 34.5 (69/2) is 34.5/180*pi
    # 21 (42/2) is 21/180*pi
    horizontal_shift = elevation * np.tan(34.5/180*np.pi)
    vertical_shift = elevation * np.tan(21/180*np.pi)

    rounded_horizontal = round(horizontal_shift)
    rounded_vertical = round(vertical_shift)
    
    if (mat[0] == 1):
        #go left some predetermined distance
        p1.left_velocity(rounded_horizontal, 1)
    else if (mat[0] == 3):
        #go right some predetermined distance
        p1.right_velocity(rounded_horizontal, 1)

    if (mat[1] == 1):
        #go up some predetermined distance
        p1.forward_velocity(rounded_vertical, 1)
    else if (mat[1] == 3):
        #go down some predetermined distance
        p1.backwards_velocity(rounded_vertical, 1)

    p1.downwards_velocity(5m)

def starting_left_pattern(): #function for starting on right side and going left
    result = evaluate_image(pic, 200, tamu_model_v5)
    if (result == False):
        print("Starting search pattern")
        result2 = False
        while (result2 == False):
            p1.left_velocity(5m)
            result2 = evaluate_image(pic, 200, tamu_model_v5)
            if (drone_location == left_boundary_point):
                break

    if (result2 == True): #logo was detected
        print("Logo area detected")
        while(drone_elevation > 1m):
            #take new picture
            shift_drone_position(pic, 600, tamu_model_v5, drone_elevation)
        VehicleMode("Land")
        return True
    elif(drone_location == left_boundary_point):
        p1.forward_velocity(2m)
        print("Reached left starting point")
        return False

def starting_right_pattern(): #function for starting on left side and going right
    result = evaluate_image(pic, 200, tamu_model_v5)
    if (result == False):
    print("Starting search pattern")
    result2 = False
    while (result2 == False):
        p1.right_velocity(5m)
        result2 = evaluate_image(pic, 200, tamu_model_v5)
        if (drone_location == right_boundary_point):
            break

    if (result2 == True): #logo was detected
        print("Logo area detected")
        while(drone_elevation > 1m):
            #take new picture
            shift_drone_position(pic, 600, tamu_model_v5, drone_elevation)
        VehicleMode("Land")
        return True
    elif(drone_location == right_boundary_point):
        p1.forward_velocity(2m)
        print("Reached right starting point")
        return False

#psuedo code

## SETTING UP REALSENSE CAMERA ##
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

frames = pipeline.wait_for_frames()
color_frame = frames.get_color_frame()

p1 = Movement()


#drone starts from initial position
p1.arm_and_takeoff(6) #assume that it starts on right side of the field
var = False
var2 = False
while ((var == False) && (var2 == False)):
    var = starting_left_pattern()
    var2 = starting_right_pattern()

        
        
#while drone is on search pattern
#   call evaluate_image function
#   if it returns true break out of while loop

#while drone elevation is above 0ft
#   call shift_drone_position function

