from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
import time
import argparse
import numpy as np
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print ('Connecting to vehicle on: %s' % args.connect)
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
            print("Current location is", self.loc)
            if self.loc[0] > 95: #This is to stop the drone while it is going through its search pattern
                print("Nearby x location")
                self.signal = 1
                break
            elif self.loc[1] > 41:
                print("Nearby y location")
                self.signal = 1
                break
            else:
                vehicle.send_mavlink(msg)
                time.sleep(1)
        print("broke out of for loop")
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
            if self.loc[0] > 95:  # This is to stop the drone while it is going through its search pattern
                print("Nearby x location")
                self.signal = 1
                break
            elif self.loc[1] > 41:
                print("Nearby y location")
                self.signal = 1
                break
            else:
                vehicle.send_mavlink(msg)
                time.sleep(1)
        print("broke out of for loop")
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
            print("Current location is", self.loc)
            if self.loc[0] > 95:  # This is to stop the drone while it is going through its search pattern
                print("Nearby x location")
                self.signal = 1
                break
            elif self.loc[1] > 41:
                print("Nearby y location")
                self.signal = 1
                break
            else:
                vehicle.send_mavlink(msg)
                time.sleep(1)
    print("broke out of for loop")
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
            print("Current location is", self.loc)
            if self.loc[0] > 95:  # This is to stop the drone while it is going through its search pattern
                print("Nearby x location")
                self.signal = 1
                break
            elif self.loc[1] > 41:
                print("Nearby y location")
                self.signal = 1
                break
            else:
                vehicle.send_mavlink(msg)
                time.sleep(1)
        print("broke out of for loop")
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

#Making array that will act as the location of the logo
logo = [100,50]
#def search_sequence():
p1 = Movement()
p1.arm_and_takeoff(20)
i = 0
Location_x_range= np.arange(90,110,1) #Defining a range for the 100 x logo location
Location_y_range=np.arange(40,60,1) #Defining the y range for the y logo location
     #Defining the starting location of the drone
    #Check if starting location is nearby the drone location
print("Vehicle location local:",vehicle.location.local_frame)
print("Here are the x range values:",Location_x_range)
#Initial Check
if int(p1.loc[0]) == np.any(Location_x_range):
        time.sleep(1)
        print("Nearby x logo location")
elif int(p1.loc[1]) == np.any(Location_y_range):
        time.sleep(1)
        print("Nearby y location")
elif int(p1.loc[0]) == np.any(Location_x_range) and int(p1.loc[1]) == np.any(Location_y_range):
        print("Nearby x and y logo, so start homing")
else:
        print("Not nearby drone location,continue to perform search pattern")
        while p1.signal!= 1:
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
        #while p1.loc:
            #current_loc = [vehicle.location.local_frame.north, vehicle.location.local_frame.east]
            #print(current_loc)
            #if current_loc[0] == Location_x_range:
               # print("Nearby x logo location")
                #break
            #elif current_loc[1] == Location_y_range:
                #print("Nearby y location")
                #break
            #elif current_loc[0] == Location_x_range and current_loc[1] == Location_y_range:
                #print("Nearby x and y logo, so start homing")
                #break
            #time.sleep(1)


    #while current_loc[0]!= logo[0] & current_loc[1]!= logo[1] :
        #Check
        #p1.forward_velocity(5,2)
        #a = abs(vehicle.location.local_frame.north-logo[0])
        """
        p1.backwards_velocity(5,2)
        p1.left_velocity(5, 2)
        b = abs(vehicle.location.local_frame.east-logo[1])
        p1.right_velocity(5,2)
        p1.right_velocity(5, 2)
        c = abs(vehicle.location.local_frame.east - logo[1])
        p1.forward_velocity(40,3)
        p1.right_velocity(25,10)
        p1.forward_velocity(40,3)
        p1.left_velocity(25,10) 
        """



