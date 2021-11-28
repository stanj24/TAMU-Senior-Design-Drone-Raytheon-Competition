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
class Movement:
    def __init__(self):
        self.velocity_x = 0
        self.velocity_y = 0
        self.velocity_z = 0
        self.duration   = 0
        self.loc = [0,0] #Test code
        self.signal = 0
        self.new_location = 0
        self.side_boundary_location = 0

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
    def starting_loc_function(self):  # To get the starting location of the drone
        global starting_x_location  # To allow the x location variable is used in all functions
        global starting_y_location  # To allow the y location variable to be used in all functions
        starting_x_location = int(vehicle.location.local_frame.north)
        starting_y = int(vehicle.location.local_frame.east)
        starting_y_location = starting_y
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

def Logo_y_search():
    if p1.loc[1] <0:
        while True:
            if p1.loc[1] >27:
                print("Nearby Logo y Location")
                break
            else:
                p1.right_velocity(1,1)
    elif p1.loc[1]>30:
        while True:
            if p1.loc[1] >26 and p1.loc[1] <30:
                print("Nearby Logo y Location")
                break
            else:
                p1.left_velocity(1,1)
    else:
        while True:
            if p1.loc[1] >27:
                print("Nearby Logo y Location")
                break
            else:
                p1.right_velocity(1,1)
print("Found x and y area of logo!")
def Search_pattern_straight():
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
def Search_pattern_left():
    p1 =Movement()
    current_y_location_ = p1.loc[1]
    while True:
            current_y_location_1 = p1.loc[1] - current_y_location_
            if current_y_location_1 >48 or current_y_location_1<-48:
                print("Drone met 50 yard boundary time to go forward")
                break
            elif p1.loc[0]>76:
                print("Nearby Logo x location")
                break
            else:
                p1.left_velocity(2, 1)
    if p1.loc[0]>76:
        print("Time to start homing in on the logo y position")
        Logo_y_search()
    else:
        Search_pattern_straight()
def Search_pattern_right():
    current_y_location_=p1.loc[1]
    while True:
        current_y_location_1=p1.loc[1]-current_y_location_
        if current_y_location_1 >48 or current_y_location_1<-48:
                print("Drone met 50 yard boundary time to go forward")
                break
        elif p1.loc[0]>76:
                print("Nearby Logo x location")
                break
        else:
                p1.right_velocity(2, 1)
    if p1.loc[0] > 76:
        print("Time to start homing in on the y position")
        Logo_y_search()
    else:
        Search_pattern_straight()
def Drone_search_pattern():
    p1 = Movement()
    p1.left_velocity(15, 10)
    p1.forward_velocity(15, 3)
    p1.right_velocity(25, 10)
    p1.forward_velocity(15, 3)
    p1.left_velocity(15, 10)
    p1.forward_velocity(15, 3)
    p1.right_velocity(25,10)
    p1.forward_velocity(15, 3)

def condition_yaw(heading, relative=False):
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
def homing_sequence_y():
    if p1.loc[1] > 30:
        while True:
            if p1.loc[1] == 29 or p1.loc[1] ==30:
                print("Y coordinate found")
                break
            else:
                p1.left_velocity(1,1)
        print("Time to land")
    elif p1.loc[1]<30:
        while True:
            if p1.loc[1] == 29 or p1.loc[1] == 30:
                print("Y coordinate found")
                break
            else:
                p1.right_velocity(1,1)
        print("Time to land")
    else:
        print("Y coordinate found")
        print("Time to land")
def homing_sequence_x():
    if p1.loc[0] > 80:
        while True:
            if p1.loc[0] ==79 or p1.loc[0] ==80:
                print("X cordinate found")
                break
            else:
                p1.backwards_velocity(1,1)
        homing_sequence_y()
    elif p1.loc[0]<80:
        while True:
            if p1.loc[0] == 79 or p1.loc[0] == 80:
                print("X cordinate found")
                break
            else:
                p1.forward_velocity(1,1)
        homing_sequence_y()
    else:
        print("X cordinate found")
        homing_sequence_y()



#Making array that will act as the location of the logo
logo = [80,30] #This logo location is defined in the bounds of 100 and 50
condition_yaw(0) #setting the drone as straight
#def search_sequence():
p1 = Movement()
p1.arm_and_takeoff(6)
p1.starting_loc_function()
i = 0
Location_x_range= np.arange(90,110,1) #Defining a range for the 100 x logo location
Location_y_range=np.arange(40,60,1) #Defining the y range for the y logo location
     #Defining the starting location of the drone
    #Check if starting location is nearby the drone location
print("Vehicle location local:",vehicle.location.local_frame)
#print("Here are the x range values:",Location_x_range)
#Initial Check
if int(p1.loc[0]) >76:
        time.sleep(1)
        print("Nearby x logo location")
elif int(p1.loc[1]) >30:
        time.sleep(1)
        print("Nearby y location")
elif int(p1.loc[0]) == np.any(Location_x_range) and int(p1.loc[1]) == np.any(Location_y_range):
        print("Nearby x and y logo, so start homing")
        homing_sequence_x()
else:
        print("Not nearby drone location,continue to perform search pattern")
        while True:
            Search_pattern_left()
            if p1.loc[0] > 78:  # This is to stop the drone while it is going through its search pattern
                print("Nearby x location")
                break
            elif p1.loc[1] >27:
                print("Nearby Logo y Location")
                break
            else:
                print("Area not found")
            Search_pattern_right()
            if p1.loc[0] > 78:  # This is to stop the drone while it is going through its search pattern
                print("Nearby x location")
                break
            elif p1.loc[1]>27:
                print("Nearby Logo y Location")
                break
            else:
                print("Area not found")
        homing_sequence_x()




"""        
        p1.left_velocity(15, 10)
            if p1.loc[0] > 95:  # This is to stop the drone while it is going through its search pattern
                print("Nearby x location")
                break
            elif p1.loc[1] > 41:
                print("Nearby y location")
                break
            else:
                print("Area not found")
            p1.forward_velocity(15, 3)
            if p1.loc[0] > 95:  # This is to stop the drone while it is going through its search pattern
                print("Nearby x location")
                break
            elif p1.loc[1] > 41:
                print("Nearby y location")
                break
            else:
                print("Area not found")
            p1.right_velocity(25, 10)
            if p1.loc[0] > 95:  # This is to stop the drone while it is going through its search pattern
                print("Nearby x location")
                break
            elif p1.loc[1] > 41:
                print("Nearby y location")
                break
            else:
                print("Area not found")
            p1.forward_velocity(15, 3)
            if p1.loc[0] > 95:  # This is to stop the drone while it is going through its search pattern
                print("Nearby x location")
                break
            elif p1.loc[1] > 41:
                print("Nearby y location")
                break
            else:
                print("Area not found")
            p1.left_velocity(15, 10)
            if p1.loc[0] > 95:  # This is to stop the drone while it is going through its search pattern
                print("Nearby x location")
                break
            elif p1.loc[1] > 41:
                print("Nearby y location")
                break
            else:
                print("Area not found")
            p1.forward_velocity(15, 3)
            if p1.loc[0] > 95:  # This is to stop the drone while it is going through its search pattern
                print("Nearby x location")
                break
            elif p1.loc[1] > 41:
                print("Nearby y location")
                break
            else:
                print("Area not found")
            p1.right_velocity(25, 10)
            if p1.loc[0] > 95:  # This is to stop the drone while it is going through its search pattern
                print("Nearby x location")
                break
            elif p1.loc[1] > 41:
                print("Nearby y location")
                break
            else:
                print("Area not found")
            p1.forward_velocity(15, 3)
            if p1.loc[0] > 95:  # This is to stop the drone while it is going through its search pattern
                print("Nearby x location")
                break
            elif p1.loc[1] > 41:
                print("Nearby y location")
                break
            else:
                print("Area not found")
        print("area found!!!!")
        time.sleep(1)
        if p1.loc[1] < 0: #This is to home in on the y-logo coordinate after getting near the x area
            while True:
                if p1.loc[1] >50:
                    break
                else:
                    print("Homing in on y coordinate")
                    p1.right_velocity(5,10)
        print("Y coordinate area found")
"""





    #while current_loc[0]!= logo[0] & current_loc[1]!= logo[1] :
        #Check
        #p1.forward_velocity(5,2)
        #a = abs(vehicle.location.local_frame.north-logo[0])




