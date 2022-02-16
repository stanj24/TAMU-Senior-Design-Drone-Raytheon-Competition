from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
import geopy
from geopy.distance import geodesic
import math
import time
import argparse
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect', default='127.0.0.1:14551')
args = parser.parse_args()
connection_string = args.connect
# Connect to the Vehicle
print ('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(args.connect, baud=921600, wait_ready=True)

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
        current_loc = vehicle.location.local_frame
        print("Current location is", current_loc)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
# Method to possibly move the lat and longitude coordinates to a different location
def translate_latlong(lat, long, lat_translation_meters, long_translation_meters):
    ''' method to move any lat,long point by provided meters in lat and long direction.
    params :
        lat,long: lattitude and longitude in degrees as decimal values, e.g. 37.43609517497065, -122.17226450150885
        lat_translation_meters: movement of point in meters in lattitude direction.
                                positive value: up move, negative value: down move
        long_translation_meters: movement of point in meters in longitude direction.
                                positive value: left move, negative value: right move
        '''
    earth_radius = 6378.137

    # Calculate top, which is lat_translation_meters above
    m_lat = (1 / ((2 * math.pi / 360) * earth_radius)) / 1000
    lat_new = lat + (lat_translation_meters * m_lat)

    # Calculate right, which is long_translation_meters right
    m_long = (1 / ((2 * math.pi / 360) * earth_radius)) / 1000 # 1 meter in degree
    long_new = long + (long_translation_meters * m_long) / math.cos(lat * (math.pi / 180))
    altitude = 0

    return long_new #Changed
def translate_up_down(lat, long, lat_translation_meters, long_translation_meters):
    ''' method to move any lat,long point by provided meters in lat and long direction.
    params :
        lat,long: lattitude and longitude in degrees as decimal values, e.g. 37.43609517497065, -122.17226450150885
        lat_translation_meters: movement of point in meters in lattitude direction.
                                positive value: up move, negative value: down move
        long_translation_meters: movement of point in meters in longitude direction.
                                positive value: left move, negative value: right move
        '''
    earth_radius = 6378.137

    # Calculate top, which is lat_translation_meters above
    m_lat = (1 / ((2 * math.pi / 360) * earth_radius)) / 1000
    lat_new = lat + (lat_translation_meters * m_lat)

    # Calculate right, which is long_translation_meters right
    m_long = (1 / ((2 * math.pi / 360) * earth_radius)) / 1000 # 1 meter in degree
    long_new = long + (long_translation_meters * m_long) / math.cos(lat * (math.pi / 180))
    altitude = 0

    return lat_new #Changed

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
#Main Code
arm_and_takeoff(10)
condition_yaw(0)
Current_location_x= vehicle.location.global_relative_frame.lat
Current_location_y= vehicle.location.global_relative_frame.lon
X_new=translate_up_down(Current_location_x,Current_location_y,30,0)
print("New Latitude Coordinates are:",X_new)
print("Going towards first point for 30 seconds ...")
point1 = LocationGlobalRelative(X_new,Current_location_y,0)
vehicle.simple_goto(point1)

# sleep so we can see the change in map

# sleep so we can see the change in map
time.sleep(10)

print("Time to land ")
vehicle.mode = VehicleMode("LAND")
vehicle.close()