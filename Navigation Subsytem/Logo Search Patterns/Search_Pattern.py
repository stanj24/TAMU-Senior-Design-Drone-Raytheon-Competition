from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
from dronekit_sitl import SITL
import time
import argparse
import numpy as np
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


#Main Code
#Making array that will act as the location of the logo
logo = [80,15] #This logo location is defined in the bounds of 100 and 50
condition_yaw(0) #setting the drone as straight
p1 = Movement()
p1.arm_and_takeoff(6) #Taking off to starting elevation of 20 feet/6 meters
p1.starting_loc_function() #Logging the starting location of the drone
i = 0
#Check if starting location is nearby the drone location
print("Vehicle starting location:",vehicle.location.local_frame)
#print("Here are the x range values:",Location_x_range)
#Initial Check
if int(p1.loc[0]) >77:#If drone is already near x logo location then go look for logo_y_location
        time.sleep(1)
        print("Nearby x logo location area")
        Logo_y_search() #Putting the drone over the correct y logo location area
        homing_sequence_x() #Homing in on the exact x and then y logo coordinates
elif int(p1.loc[1])>12 and int(p1.loc[1])<=15:#If the drone is in the y logo location area
    print("Nearby y logo location are")
    homing_sequence_x() #Hones in on the x location coordicate and then the y for the logo
elif int(p1.loc[0]) == 80 and int(p1.loc[1]) == 15: #If the drone is already over the logo then it does it checks then lands
        print("On the x and y logo, so start homing")
        homing_sequence_y()
else: #This states the the drone is not near the x and y logo areas
        print("Not nearby drone location,continue to perform search pattern")
        while True:
            Search_pattern_left()
            if p1.loc[0] > 78:  # This is to stop the drone while it is going through its search pattern
                print("Nearby x location")
                break
            else:
                print("Area not found")
            Search_pattern_right()
            if p1.loc[0] > 78:  # This is to stop the drone while it is going through its search pattern
                print("Nearby x location")
                break
            else:
                print("Area not found")
        homing_sequence_x()






