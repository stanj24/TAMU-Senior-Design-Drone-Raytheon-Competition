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

import pyrealsense2 as rs
import numpy as np
import cv2
import os
import matplotlib.pyplot as plt

import math
import time
import argparse
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect', default='127.0.0.1:14551')
args = parser.parse_args()
connection_string = args.connect

img_og = cv2.imread("TAM-LogoBox.jpg")

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

#Stanley's Image Recognition Functions

#Stanley's taking an image function
def take_image(): #pipeline must be started before calling
    time.sleep(3)
    #get color frame
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()

    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
    return color_image

def evaluate_image(img1, img2):
    # Initiate SIFT detector
    sift = cv2.SIFT_create()

    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(img1,None)
    kp2, des2 = sift.detectAndCompute(img2,None)

    # BFMatcher with default params
    bf = cv2.BFMatcher()
    try:
        matches = bf.knnMatch(des1,des2,k=2)
    except:
        return 0

    # Apply ratio test
    mat = 0
    good = []
    for m,n in matches:
        if m.distance < 0.75*n.distance:
            good.append([m])
            mat = mat + 1
    # cv.drawMatchesKnn expects list of lists as matches.
    # img3 = cv.drawMatchesKnn(img1,kp1,img2,kp2,good,None,flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    
    return mat #returns number of matching pixels

def forward_GPS_point(rounded_vertical): #New Going Forward Function - Jeremiah 
    Current_location_x_new = vehicle.location.global_relative_frame.lat  # Getting the starting poitns
    Current_location_y_new = vehicle.location.global_relative_frame.lon
    New_straight_point = translate_up_down(Current_location_x_new, Current_location_y_new, rounded_vertical, 0)
    print("Flying Straight")
    Altitude = vehicle.location.global_relative_frame.alt
    Travel_point_2 = LocationGlobalRelative(New_straight_point, Current_location_y_new, Altitude)
    vehicle.simple_goto(Travel_point_2)
    time.sleep(10)

def backwards_GPS_point(rounded_vertical): #New Going Function - Jeremiah 
    Current_location_x_new = vehicle.location.global_relative_frame.lat  # Getting the starting poitns
    Current_location_y_new = vehicle.location.global_relative_frame.lon
    New_straight_point = translate_up_down(Current_location_x_new, Current_location_y_new, -1*rounded_vertical, 0)
    print("Flying Backwards")
    Altitude = vehicle.location.global_relative_frame.alt
    Travel_point_2 = LocationGlobalRelative(New_straight_point, Current_location_y_new, Altitude)
    vehicle.simple_goto(Travel_point_2)
    time.sleep(10)

def left_GPS_point(rounded_horizontal): #New Going Function - Jeremiah 
    Current_location_x_new = vehicle.location.global_relative_frame.lat  # Getting the starting poitns
    Current_location_y_new = vehicle.location.global_relative_frame.lon
    Longitude_new = translate_latlong(Current_location_x_new, Current_location_y_new, 0, -1*rounded_horizontal)
    print("Flying left")
    Altitude = vehicle.location.global_relative_frame.alt
    Travel_point = LocationGlobalRelative(Current_location_x_new, Longitude_new, Altitude)
    vehicle.simple_goto(Travel_point)
    time.sleep(10)

def right_GPS_point(rounded_horizontal): #New Going Function - Jeremiah 
    Current_location_x_new = vehicle.location.global_relative_frame.lat  # Getting the starting poitns
    Current_location_y_new = vehicle.location.global_relative_frame.lon
    Longitude_new = translate_latlong(Current_location_x_new, Current_location_y_new, 0, rounded_horizontal)
    print("Flying right")
    Altitude = vehicle.location.global_relative_frame.alt
    Travel_point = LocationGlobalRelative(Current_location_x_new, Longitude_new, Altitude)
    vehicle.simple_goto(Travel_point)
    time.sleep(10)

#Rough draft function for going left
def Search_Pattern_Left():
    #this is an initial check before we start to go left
    #take picture
    pic = take_image()
    val = evaluate_image(img_og, pic)
    if (val > 15): #if pixel count is greater than 15, start homing
        print("Time to land")
        vehicle.mode = VehicleMode("LAND")
        vehicle.close()

    x = 0
    while(x < 5):
        left_GPS_point(5)
        time.sleep(5)

        #take picture
        pic = take_image()
        val = evaluate_image(img_og, pic)
        if (val > 15): #if pixel count is greater than 15, start homing
            print("Time to land")
            vehicle.mode = VehicleMode("LAND")
            vehicle.close()
            break
        x = x+1
    
    if (x == 5): #reached end of field, time to go straight
        forward_GPS_point(3)
    else:
        print("Error or we landed")

def Search_Pattern_Right():
    #this is an initial check before we start to go right
    #take picture
    pic = take_image()
    val = evaluate_image(img_og, pic)
    if (val > 15): #if pixel count is greater than 15, start homing
        print("Time to land")
        vehicle.mode = VehicleMode("LAND")
        vehicle.close()

    x = 0
    while(x < 5):
        right_GPS_point(5)
        time.sleep(5)

        #take picture
        pic = take_image()
        val = evaluate_image(img_og, pic)
        if (val > 15): #if pixel count is greater than 15, start homing
            print("Time to land")
            vehicle.mode = VehicleMode("LAND")
            vehicle.close()
            break
        x = x+1
    
    if (x == 5): #reached end of field, time to go straight
        Current_location_x_new = vehicle.location.global_relative_frame.lat  # Getting the starting poitns
        Current_location_y_new = vehicle.location.global_relative_frame.lon
        Longitude_new = translate_up_down(Current_location_x_new, Current_location_y_new, 3, 0)
        print("Flying forward")
        Travel_point = LocationGlobalRelative(Longitude_new, Current_location_y_new, 0)
        vehicle.simple_goto(Travel_point)
    else:
        print("Error or we landed")


#coords_1 = (52.2296756, 21.0122287)
#coords_2 = (52.406374, 16.9251681)

#print(geopy.distance.geodesic(coords_1, coords_2).m) #Print out the distance in meters between two points

# main code
arm_and_takeoff(7)
condition_yaw(0)
#Starting_location_x= vehicle.location.global_relative_frame.lat
#Starting_location_y= vehicle.location.global_relative_frame.lon
#Starting_travel_coordinates = (Starting_location_x, 0)

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 8)
# Start streaming
pipeline.start(config)

a = input("Please enter distance from the right side of the field (in meters): ")
right_GPS_point(a)

#starting on the right side
for i in range(3):
    Search_Pattern_Left()
    Search_Pattern_Right()


pipeline.stop()



