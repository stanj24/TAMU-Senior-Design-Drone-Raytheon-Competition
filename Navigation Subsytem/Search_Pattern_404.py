#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
simple_goto.py: GUIDED mode "simple goto" example (Copter Only)

Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.

Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""
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


coords_1 = (52.2296756, 21.0122287)
coords_2 = (52.406374, 16.9251681)

print(geopy.distance.geodesic(coords_1, coords_2).m) #Print out the distance in meters between two points

arm_and_takeoff(10)

Current_location_x= vehicle.location.global_relative_frame.lat
Current_location_y= vehicle.location.global_relative_frame.lon

Starting_coordinates = (Current_location_x,0)
print("This is the initial lat and long coordinates",Current_location_x,Current_location_y)
print("Set default/target airspeed to 3")
vehicle.airspeed = 3
#Search pattern loop, staring with going left
Current_location_x_new= vehicle.location.global_relative_frame.lat #Getting the starting poitns
Current_location_y_new= vehicle.location.global_relative_frame.lon
Traveled_coordinates =(Current_location_x_new,0)
Distance_traveled_straight= geopy.distance.geodesic(Starting_coordinates,Traveled_coordinates).m
print("Distance Traveled in x-direction",Distance_traveled_straight)
Longitude_new=translate_latlong(Current_location_x_new,Current_location_y_new,0,-50)
print("Flying left")
Travel_point= LocationGlobalRelative(Current_location_x_new,Longitude_new,0)
vehicle.simple_goto(Travel_point)
time.sleep(10)

#Going straight
Current_location_x_new= vehicle.location.global_relative_frame.lat #Getting the starting poitns
Current_location_y_new= vehicle.location.global_relative_frame.lon
Traveled_coordinates =(Current_location_x_new,0)
Distance_traveled_straight= geopy.distance.geodesic(Starting_coordinates,Traveled_coordinates).m
print("Distance Traveled in x-direction",Distance_traveled_straight)
New_straight_point=translate_up_down(Current_location_x_new,Current_location_y_new,2,0)
print("Flying Straight")
Travel_point_2= LocationGlobalRelative(New_straight_point,Current_location_y_new,0)
vehicle.simple_goto(Travel_point_2)
time.sleep(10)

#Going right
Current_location_x_new= vehicle.location.global_relative_frame.lat #Getting the starting poitns
Current_location_y_new= vehicle.location.global_relative_frame.lon
Traveled_coordinates =(Current_location_x_new,0)
Distance_traveled_straight= geopy.distance.geodesic(Starting_coordinates,Traveled_coordinates).m
print("Distance Traveled in x-direction",Distance_traveled_straight)
Longitude_new=translate_latlong(Current_location_x_new,Current_location_y_new,0,50)
print("Flying right")
Travel_point= LocationGlobalRelative(Current_location_x_new,Longitude_new,0)
vehicle.simple_goto(Travel_point)
time.sleep(10)





Long_new=translate_latlong(Current_location_x,Current_location_y,0,-30)
print("New Longitude Coordinates are:",Long_new)
print("Going towards first point for 30 seconds ...")
point1 = LocationGlobalRelative(Current_location_x,Long_new,0)
vehicle.simple_goto(point1)

# sleep so we can see the change in map

# sleep so we can see the change in map
time.sleep(10)

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()


