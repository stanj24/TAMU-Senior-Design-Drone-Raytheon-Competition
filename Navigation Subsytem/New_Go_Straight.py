
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
vehicle = connect(args.connect, baud=57600, wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(5)

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


def condition_yaw(heading, relative=False): #This function is to set the drone to a certain heading
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

#Functions for moving in all directions 
def Move_Left(Distance_to_move):
    Current_location_x_new = vehicle.location.global_relative_frame.lat  # Getting the starting poitns
    Current_location_y_new = vehicle.location.global_relative_frame.lon
    Longitude_new = translate_latlong(Current_location_x_new, Current_location_y_new, 0, -1*Distance_to_move)
    print("Flying left")
    Altitude = vehicle.location.global_relative_frame.alt
    Travel_point = LocationGlobalRelative(Current_location_x_new, Longitude_new, Altitude)
    vehicle.simple_goto(Travel_point)
    time.sleep(20)
def Move_Right(Distance_to_move):
    Current_location_x_new = vehicle.location.global_relative_frame.lat  # Getting the starting poitns
    Current_location_y_new = vehicle.location.global_relative_frame.lon
    Longitude_new = translate_latlong(Current_location_x_new, Current_location_y_new, 0, Distance_to_move)
    print("Flying right")
    Altitude = vehicle.location.global_relative_frame.alt
    Travel_point = LocationGlobalRelative(Current_location_x_new, Longitude_new, Altitude)
    vehicle.simple_goto(Travel_point)
    time.sleep(20)
def Move_Straight(Distance_to_move):
    Current_location_x_new = vehicle.location.global_relative_frame.lat  # Getting the starting points
    Current_location_y_new = vehicle.location.global_relative_frame.lon
    New_straight_point = translate_up_down(Current_location_x_new, Current_location_y_new, Distance_to_move, 0)
    print("Flying Straight")
    Altitude = vehicle.location.global_relative_frame.alt
    Travel_point_2 = LocationGlobalRelative(New_straight_point, Current_location_y_new, Altitude)
    vehicle.simple_goto(Travel_point_2)
    time.sleep(20)
def Move_Backwards(Distance_to_move):
    Current_location_x_new = vehicle.location.global_relative_frame.lat  # Getting the starting poitns
    Current_location_y_new = vehicle.location.global_relative_frame.lon
    New_straight_point = translate_up_down(Current_location_x_new, Current_location_y_new, -1*Distance_to_move, 0)
    print("Flying Backwards")
    Altitude = vehicle.location.global_relative_frame.alt
    Travel_point_2 = LocationGlobalRelative(New_straight_point, Current_location_y_new, Altitude)
    vehicle.simple_goto(Travel_point_2)
    time.sleep(20)

#Functions to return the new latitude and longitude points by inputting in starting points and the heading in degrees  
def Move_new_x(Distance_to_move,b):
    Current_location_x_new = vehicle.location.global_relative_frame.lat  # Getting the starting poitns
    Current_location_y_new = vehicle.location.global_relative_frame.lon
    origin = geopy.Point(Current_location_x_new,Current_location_y_new)
    destination = geopy.distance.geodesic(meters=Distance_to_move).destination(origin, b)
    lat2= destination.latitude
    return lat2

def Move_new_y(Distance_to_move,b):
    Current_location_x_new = vehicle.location.global_relative_frame.lat  # Getting the starting poitns
    Current_location_y_new = vehicle.location.global_relative_frame.lon
    origin = geopy.Point(Current_location_x_new,Current_location_y_new)
    destination = geopy.distance.geodesic(meters=Distance_to_move).destination(origin, b)
    lon2 = destination.longitude
    return lon2

coords_1 = (52.2296756, 21.0122287)
coords_2 = (52.406374, 16.9251681)
print(geopy.distance.geodesic(coords_1, coords_2).m) #Print out the distance in meters between two points, TEST CODE
#Main Code 
arm_and_takeoff(10) #Drone taking off to certain altitude measured in meters
condition_yaw(45) #Setting the drone in a specified direction/heading measured in degrees from mission planner
Current_location_x= vehicle.location.global_relative_frame.lat
Current_location_y= vehicle.location.global_relative_frame.lon
New_straight_point_x = Move_new_x(30,45) #Finding new latitude points by inputting the distance to travel (meters) and direction (degrees)
New_straight_point_y= Move_new_y(30,45) #Finding new longitude points 
print("Flying Straight")
Altitude = vehicle.location.global_relative_frame.alt
Travel_point_2 = LocationGlobalRelative(New_straight_point_x,New_straight_point_y,7) #Setting flight travel points
vehicle.simple_goto(Travel_point_2) #Drone flying to travel point
time.sleep(20) #Hover for specified time in seconds 
print("Time to land ")
vehicle.mode = VehicleMode("LAND") #Landing call
vehicle.close()
