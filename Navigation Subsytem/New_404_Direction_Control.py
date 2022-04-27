
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
#Function to find new latitiude point 
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
def condition_yaw(heading, relative=False): #This function is to set the drone heading 
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

#Rough draft function for going left
def Move_Left(Distance_to_move):
    Current_location_x_new = vehicle.location.global_relative_frame.lat  # Getting the starting poitns
    Current_location_y_new = vehicle.location.global_relative_frame.lon
    Longitude_new = translate_latlong(Current_location_x_new, Current_location_y_new, 0, -1*Distance_to_move) #Function finding new longitude point 
    print("Flying left")
    Altitude = vehicle.location.global_relative_frame.alt
    #Function defining new travel point that will use the same latitude point but different longitude point in order to move left 
    Travel_point = LocationGlobalRelative(Current_location_x_new, Longitude_new, 7) #Putting in the new latitude, longitude, and altitude in order for the drone to travel to
    vehicle.simple_goto(Travel_point) #Telling the drone to go to the new point 
    time.sleep(10) #Drone hover 
def Move_Right(Distance_to_move):
    Current_location_x_new = vehicle.location.global_relative_frame.lat  # Getting the starting poitns
    Current_location_y_new = vehicle.location.global_relative_frame.lon
    Longitude_new = translate_latlong(Current_location_x_new, Current_location_y_new, 0, Distance_to_move)
    print("Flying right")
    Altitude = vehicle.location.global_relative_frame.alt
    Travel_point = LocationGlobalRelative(Current_location_x_new, Longitude_new, 7)
    vehicle.simple_goto(Travel_point)
    time.sleep(10)
def Move_Straight(Distance_to_move):
    Current_location_x_new = vehicle.location.global_relative_frame.lat  # Getting the starting points
    Current_location_y_new = vehicle.location.global_relative_frame.lon
    New_straight_point = translate_up_down(Current_location_x_new, Current_location_y_new, Distance_to_move, 0)
    print("Flying Straight")
    Altitude = vehicle.location.global_relative_frame.alt
    Travel_point_2 = LocationGlobalRelative(New_straight_point, Current_location_y_new, 7)
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
    time.sleep(10)
def Move_Downwards(Distance_to_move):
    Current_location_x_new = vehicle.location.global_relative_frame.lat  # Getting the starting poitns
    Current_location_y_new = vehicle.location.global_relative_frame.lon
    #New_straight_point = translate_up_down(Current_location_x_new, Current_location_y_new, -1 * Distance_to_move, 0)
    print("Flying Downwards")
    Altitude = vehicle.location.global_relative_frame.alt
    Altitude_new = Altitude - Distance_to_move
    Travel_point_2 = LocationGlobalRelative(Current_location_x_new, Current_location_y_new, Altitude_new)
    vehicle.simple_goto(Travel_point_2)
    time.sleep(10)
def Move_Upwards(Distance_to_move):
    Current_location_x_new = vehicle.location.global_relative_frame.lat  # Getting the starting poitns
    Current_location_y_new = vehicle.location.global_relative_frame.lon
    New_straight_point = translate_up_down(Current_location_x_new, Current_location_y_new, -1 * Distance_to_move, 0)
    print("Flying Backwards")
    Altitude = vehicle.location.global_relative_frame.alt
    Travel_point_2 = LocationGlobalRelative(New_straight_point, Current_location_y_new, Altitude)
    vehicle.simple_goto(Travel_point_2)
    time.sleep(10)



#coords_1 = (52.2296756, 21.0122287)
#coords_2 = (52.406374, 16.9251681)

#print(geopy.distance.geodesic(coords_1, coords_2).m) #Print out the distance in meters between two points

arm_and_takeoff(7) #Telling drone to take off to specified altitude 
#Getting starting coordinates
Current_location_x= vehicle.location.global_relative_frame.lat
Current_location_y= vehicle.location.global_relative_frame.lon
Starting_coordinates=(Current_location_x,0)
#Moving left and Right
Move_Left(20) #Moving to a point 20 meters left from starting location
Move_Right(20) #Moving to a point 20 meters right from previous location
print("Time to land")
vehicle.mode = VehicleMode("LAND") #Drone landing call 
vehicle.close()
#Moving straight and backwards code
'''
Move_Straight(28)
New_Current_location_x=vehicle.location.global_relative_frame.lat
New_Current_location_y=vehicle.location.global_relative_frame.lon
Traveled_X_direction_coord = (New_Current_location_x,0)
print("Distance Traveled in X Direction:")
print(geopy.distance.geodesic(Starting_coordinates, Traveled_X_direction_coord).m) #Printing out the drone distance traveled in meters 
Move_Backwards(28)
New_Current_location_x_1=vehicle.location.global_relative_frame.lat
New_Current_location_y_1=vehicle.location.global_relative_frame.lon
Traveled_X_direction_coord_1 = (New_Current_location_x_1,0)
print("Distance Traveled in X Direction:")
print(geopy.distance.geodesic(Traveled_X_direction_coord, Traveled_X_direction_coord_1).m)
print("Time to land")
vehicle.mode = VehicleMode("LAND")
vehicle.close()
'''
