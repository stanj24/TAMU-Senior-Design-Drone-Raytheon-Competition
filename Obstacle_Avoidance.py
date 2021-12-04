from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
from dronekit_sitl import SITL
import time
import argparse
import numpy as np
import sys
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
class Continue_movement:
    def __init__(self):
        self.velocity_x = 0
        self.velocity_y = 0
        self.velocity_z = 0
        self.current_loc = [0, 0]
        self.new_location = 0 #This is to make sure the drone doesnt travel more than 100 yards in the forward direction of the field
        self.side_boundary_location =0 #This is to make sure the drone doesnt travel off of the 50 yard field boundary
    def starting_loc_function(self): #To get the starting location of the drone
        global starting_x_location  # To allow the x location variable is used in all functions
        global starting_y_location  # To allow the y location variable to be used in all functions
        starting_x_location = int(vehicle.location.local_frame.north)
        starting_y = int(vehicle.location.local_frame.east)
        starting_y_location=starting_y
    def forward_velocity(self, velocity_x_input, duration_input):
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
            x_loc = int(vehicle.location.local_frame.north)
            y_loc = int(vehicle.location.local_frame.east)
            self.current_loc = [x_loc, y_loc]
            print("Current location is", self.current_loc)
            self.new_location=self.current_loc[0]-starting_x_location #This is to update the distance traveled in the x axis
            print("X Distance Traveled :",self.new_location)
            self.side_boundary_location=self.current_loc[1]-starting_y_location #This is to make sure the drone does not travel out of the 50 yard boundary of the field
            print("Boundary Condition  :",self.side_boundary_location)
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
            self.current_loc = [starting_x_loc, starting_y_loc]
            self.new_location = self.current_loc[0] - starting_x_location
            print("Current location is", self.current_loc)
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
            self.current_loc = [starting_x_loc, starting_y_loc]
            print("Current location is", self.current_loc)
            self.new_location = self.current_loc[0] - starting_x_location
            self.side_boundary_location = self.current_loc[1] - starting_y_location
            print(" X Distance Traveled ", self.new_location)
            print("Boundary Condition:", self.side_boundary_location)
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
            self.current_loc = [starting_x_loc, starting_y_loc]
            print("Current location is", self.current_loc)
            self.new_location = self.current_loc[0] - starting_x_location
            self.side_boundary_location = self.current_loc[1] - starting_y_location
            print(" X Distance Traveled ", self.new_location)
            print("Boundary Condition:", self.side_boundary_location)
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
            starting_x_loc = int(vehicle.location.local_frame.north)
            starting_y_loc = int(vehicle.location.local_frame.east)
            self.current_loc = [starting_x_loc, starting_y_loc]
            self.new_location = self.current_loc[0] - starting_x_location
            self.side_boundary_location = self.current_loc[1] - starting_y_location
            print("Current location is", self.current_loc)
            print(" X Distance Traveled ",self.new_location)
            print("Boundary Condition:", self.side_boundary_location)
            vehicle.send_mavlink(msg)
            time.sleep(1)
    def left_diagonol_velocity(self, velocity_diagonol_input,duration_input):
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
            print("Flying left diagonal")
            starting_x_loc = int(vehicle.location.local_frame.north)
            starting_y_loc = int(vehicle.location.local_frame.east)
            self.current_loc = [starting_x_loc, starting_y_loc]
            print("Current location is", self.current_loc)
            self.new_location = self.current_loc[0] - starting_x_location
            self.side_boundary_location = self.current_loc[1] - starting_y_location
            print(" X Distance Traveled ", self.new_location)
            print("Boundary Condition:", self.side_boundary_location)
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
            starting_x_loc = int(vehicle.location.local_frame.north)
            starting_y_loc = int(vehicle.location.local_frame.east)
            self.current_loc = [starting_x_loc, starting_y_loc]
            self.new_location = self.current_loc[0] - starting_x_location
            print("Current location is", self.current_loc)
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
            starting_x_loc = int(vehicle.location.local_frame.north)
            starting_y_loc = int(vehicle.location.local_frame.east)
            self.current_loc = [starting_x_loc, starting_y_loc]
            self.new_location = self.current_loc[0] - starting_x_location
            print("Current location is", self.current_loc)
            vehicle.send_mavlink(msg)
            time.sleep(1)
def condition_yaw(heading, relative=False): #This function is to set the drone as straight
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
def obstacle_identified_function(): #Function to handle the case that and obstacle has been identified in the drone flight path
    input_answer_j = int(input("Type in obstacle location: 2-obstacle to the right, 3- obstacle to the left, 4-obstacle is directly in front\n"))
    if input_answer_j==2 or input_answer_j==3:
        print("Drone can proceed forward")
        second_iteration_code()
    else:
        input_answer_j2 = int(input("Is it best to move to the left or the right, 1 for left and 0 for right\n"))
        if input_answer_j2 == 1: #Choosing to move left
            input_answer_j2_a=int(input("Type in 1 for Sharp Horizontal left and 0 for Diagonal left \n"))
            if input_answer_j2_a==0:#Moving left diagonally
                while True:
                    if p1.new_location>98:
                        print("100 yard threshold met")
                        print("Now let's land")
                        vehicle.mode = VehicleMode("LAND")
                        vehicle.close()
                        sys.exit()
                    elif p1.side_boundary_location <= -25:  # Making sure not to exceed 50 yard boundary from the center of the field
                        print("Must proceed straight have met the 50 yard boundary")
                        break
                    while_loop_stopper_b = int(input("Is it clear to proceed forward, 1 yes and 0 no\n"))
                    if while_loop_stopper_b == 1:
                        print("Clear to proceed straight")
                        break
                    else:
                        print("Continue moving left")
                        p1.left_diagonol_velocity(1, 2)
                        continue
                second_iteration_code_b()
            else: #Drone moving sharp left horizontal
                while True:
                    if p1.new_location>98:
                        print("100 yard threshold met")
                        print("Now let's land")
                        vehicle.mode = VehicleMode("LAND")
                        vehicle.close()
                        sys.exit()
                    elif p1.side_boundary_location <= -25:  # Making sure not to exceed 50 yard boundary from the center of the field
                        print("Must proceed straight have met the 50 yard boundary")
                        break
                    while_loop_stopper_b_a = int(input("Is it clear to proceed forward, 1 yes and 0 no\n"))
                    if while_loop_stopper_b_a == 1:
                        print("Clear to proceed straight")
                        break
                    else:
                        print("Continue moving left")
                        p1.left_velocity(1, 2)
                        continue
                second_iteration_code_b()
        else: #Choose to move right if you dont type in 1
            input_answer_j2_a1 = int(input("Type in 1 for Sharp Horizontal right and 0 for Diagonal right \n"))
            if input_answer_j2_a1==0:
                while True:
                    if p1.new_location > 98:
                        print("100 yard threshold met")
                        print("Now let's land")
                        vehicle.mode = VehicleMode("LAND")
                        vehicle.close()
                        sys.exit()
                    elif p1.side_boundary_location >= 25:  # Making sure not to exceed 50 yard boundary from the center of the field
                        print("Musts proceed straight have met the 50 yard boundary")
                        break
                    while_loop_stopper_b2 = int(input("Is it clear to proceed forward, 1 yes and 0 no\n"))
                    if while_loop_stopper_b2 == 1:
                        print("Clear to proceed straight")
                        break
                    else:
                        print("Continue moving right")
                        p1.right_diagonol_velocity(1, 2)
                        continue
                second_iteration_code_b()
            else: #Case where the drone moves sharp right
                while True:
                    if p1.new_location > 98:
                        print("100 yard threshold met")
                        print("Now let's land")
                        vehicle.mode = VehicleMode("LAND")
                        vehicle.close()
                        sys.exit()
                    elif p1.side_boundary_location >= 25:  # Making sure not to exceed 50 yard boundary from the center of the field
                        print("Musts proceed straight have met the 50 yard boundary")
                        break
                    while_loop_stopper_b2 = int(input("Is it clear to proceed forward, 1 yes and 0 no\n"))
                    if while_loop_stopper_b2 == 1:
                        print("Clear to proceed straight")
                        break
                    else:
                        print("Continue moving right")
                        p1.right_velocity(1, 2)
                        continue
                second_iteration_code_b()

def second_iteration_code(): #Code to handle case where there is not an initial obstacle identified in the drone flight path
    p1.forward_velocity(3, 2)
    if p1.new_location > 98:  # Checking to see if the location exceeds the 100 yard mark
        print("Reached 100 yard threshold")
        print("Now let's land")
        vehicle.mode = VehicleMode("LAND")
        vehicle.close()
        sys.exit()
    else:
        while True:
            print("Is there an obstacle nearby, type in 1 for yes and 0 for no")
            input_answer_1 = int(input())
            if input_answer_1 == 0:  # Case that goes if there is no intitial obstacle nearby when the drone goes forward
                if p1.new_location > 98:
                    print("100 yard threshold met")
                    print("Now let's land")
                    vehicle.mode = VehicleMode("LAND")
                    vehicle.close()
                    sys.exit()
                else:
                    p1.forward_velocity(1, 2)
            elif input_answer_1 == 1:
                break
            else:
                print("User Error, try again")
                continue
        obstacle_identified_function()
def second_iteration_code_b(): #This is to help when it comes to the drone wanting to do a zig zag pattern
    if p1.new_location >= 98:  # Checking to see if the location exceeds the 100 yard mark
        print("Reached 100 yard threshold")
        print("Now let's land")
        vehicle.mode = VehicleMode("LAND")
        vehicle.close()
        sys.exit()
    else:
        while True:
            print("Is there an obstacle nearby, type in 1 for yes and 0 for no")
            input_answer_1 = int(input())
            if input_answer_1 == 0:  # Case that goes if there is no intitial obstacle nearby when the drone goes forward
                if p1.new_location >= 98:
                    print("100 yard threshold met")
                    print("Now let's land")
                    vehicle.mode = VehicleMode("LAND")
                    vehicle.close()
                    sys.exit()
                else:
                    p1.forward_velocity(1, 2)
            elif input_answer_1 == 1:
                break
            else:
                print("User Error, try again")
                continue
        obstacle_identified_function()


####Main Code
arm_and_takeoff(6) #Taking off to the minimum required altitude
p1 = Continue_movement() #Making the object from the continue movement class
p1.starting_loc_function() #Logging in the starting location of the drone
starting_x_loc = int(vehicle.location.local_frame.north)
starting_y_loc = int(vehicle.location.local_frame.east)
print("Starting x location is:",starting_x_loc)
print("Starting y location is:",starting_y_loc)
condition_yaw(0) #Setting the drone straight
print("Beginning obstacle avoidance sequence")
print("Is there any obstacles on the field at all? Type in 1 for yes and 0 for no")
input_answer_0=int(input())
new_location = p1.current_loc[0] - starting_x_loc
if input_answer_0==0: #Case:If there is no obstacles at all on the field
    while True:
        if new_location >=98:
            break
        p1.forward_velocity(3,1)
        new_location = p1.current_loc[0] - starting_x_loc
    print("Reached 100 yard threshold")
    print("Now let's land")
    vehicle.mode = VehicleMode("LAND")
    vehicle.close()
elif input_answer_0==1: #Case where the camera indicates that there are obstacles initially on the field
    if new_location >=98: #Checking to see if the location exceeds the 100 yard mark
        print("Reached 100 yard threshold")
        print("100 yard threshold met")
        print("Now let's land")
        vehicle.mode = VehicleMode("LAND")
        vehicle.close()
        sys.exit()
    else:
        print("Is there an obstacle nearby, type in 1 for yes and 0 for no")
        input_answer_1= int(input())
        if input_answer_1==0:#Case that goes if there is no intitial obstacle nearby when the drone goes forward
            second_iteration_code()
##End of the main case 1 code
        elif input_answer_1==1: #Case that there is an obstacle in front of the drone after it goes forward initially
            obstacle_identified_function()

        else:
            print("User input error, you done messed up bro")
else:
    print("User input error, you messed bro")







