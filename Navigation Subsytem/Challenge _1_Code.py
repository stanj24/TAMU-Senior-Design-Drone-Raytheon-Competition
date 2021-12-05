from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
import time
import argparse
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
class Continue_movement: #This class is used to travel certain distance forward then stop once the distance is reached
    def __init__(self):
        self.velocity_x = 0
        self.velocity_y = 0
        self.velocity_z = 0
    def forward_velocity(self, velocity_x_input):
        self.velocity_x = velocity_x_input
        self.velocity_y = 0
        self.velocity_z = 0
        print("Type in target location in yards")
        target_input = input() #Asking input statement from user about yardage
        target_input_new= float(target_input) * .9144 #Changing yards to meters
        i=0
        starting_x_loc = vehicle.location.local_frame.north
        print("Starting x_location:",starting_x_loc)
        print("The target is",target_input_new,"meters")
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
        while i< target_input_new:#Loops the checks if the inputted location matches the drones current location
            print("Current Location is",vehicle.location.local_frame.north)
            print("Flying forwards")
            vehicle.send_mavlink(msg)
            new_location=vehicle.location.local_frame.north
            i = new_location-starting_x_loc
            print("X distance traveled in meters", i)
            time.sleep(1)
#Main Code
arm_and_takeoff(6) #Taking off to starting elevation of 6 meters or 20 feet
p1 = Continue_movement()
p1.forward_velocity(1) #Traveling 1m/s forward with the while loop going
print("30 yard mark reached")
print("Now let's land")
vehicle.mode = VehicleMode("LAND") #Telling the drone to land
# Close vehicle object
vehicle.close()