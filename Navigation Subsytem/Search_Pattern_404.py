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

#Rough draft function for going left
def Search_Pattern_Left():
    x=0
    for x in range(0,24):
        Current_location_x_new = vehicle.location.global_relative_frame.lat  # Getting the starting poitns
        Current_location_y_new = vehicle.location.global_relative_frame.lon
        Longitude_new = translate_latlong(Current_location_x_new, Current_location_y_new, 0, -2)
        print("Flying left")
        Travel_point = LocationGlobalRelative(Current_location_x_new, Longitude_new, 0)
        vehicle.simple_goto(Travel_point)
        time.sleep(2)
        x+=1

#Stanley's Image Recognition Functions
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

def crop_and_analyze(image_array):

    height = image_array.shape[0]
    width = image_array.shape[1]
    
    #1-1
    left = 0
    top = 0
    right = int(width / 3)
    bottom = int(height / 3)
    im1 = image_array[top:bottom, left:right]
    pred1 = evaluate_image(img_og, im1)
    plt.imshow(im1)
    plt.show()
    print("Section 1-1")
    print(pred1)
    
    #1-2
    left = int(width / 3)
    top = 0
    right = int(2 * width / 3)
    bottom = int(height / 3)
    im2 = image_array[top:bottom, left:right]
    pred2 = evaluate_image(img_og, im2)
    plt.imshow(im2)
    plt.show()
    print("Section 1-2")
    print(pred2)
    
    #1-3
    left = int(2 * width / 3)
    top = 0
    right = width
    bottom = int(height / 3)
    im3 = image_array[top:bottom, left:right]
    pred3 = evaluate_image(img_og, im3)
    plt.imshow(im3)
    plt.show()
    print("Section 1-3")
    print(pred3)
    
    #2-1
    left = 0
    top = int(height / 3)
    right = int(width / 3)
    bottom = int(2 * height / 3)
    im4 = image_array[top:bottom, left:right]
    pred4 = evaluate_image(img_og, im4)
    plt.imshow(im4)
    plt.show()
    print("Section 2-1")
    print(pred4)
    
    #2-2
    left = int(width / 3)
    top = int(height / 3)
    right = int(2 * width / 3)
    bottom = int(2 * height / 3)
    im5 = image_array[top:bottom, left:right]
    pred5 = evaluate_image(img_og, im5)
    plt.imshow(im5)
    plt.show()
    print("Section 2-2")
    print(pred5)
    
    #2-3
    left = int(2 * width / 3)
    top = int(height / 3)
    right = width
    bottom = int(2 * height / 3)
    im6 = image_array[top:bottom, left:right]
    pred6 = evaluate_image(img_og, im6)
    plt.imshow(im6)
    plt.show()
    print("Section 2-3")
    print(pred6)
    
    #3-1
    left = 0
    top = int(2 * height / 3)
    right = int(width / 3)
    bottom = height
    im7 = image_array[top:bottom, left:right]
    pred7 = evaluate_image(img_og, im7)
    plt.imshow(im7)
    plt.show()
    print("Section 3-1")
    print(pred7)
    
    #3-2
    left = int(width / 3)
    top = int(2 * height / 3)
    right = int(2 * width / 3)
    bottom = height
    im8 = image_array[top:bottom, left:right]
    pred8 = evaluate_image(img_og, im8)
    plt.imshow(im8)
    plt.show()
    print("Section 3-2")
    print(pred8)
    
    #3-3
    left = int(2 * width / 3)
    top = int(2 * height / 3)
    right = width
    bottom = height
    im9 = image_array[top:bottom, left:right]
    pred9 = evaluate_image(img_og, im9)
    plt.imshow(im9)
    plt.show()
    print("Section 3-3")
    print(pred9)
    
    #calculate region with the highest probablity of having the logo
    maximum = max(pred1, pred2, pred3, pred4, pred5, pred6, pred7, pred8, pred9)
    
    if(pred1 == maximum):
        return [1, 1], pred1
    elif(pred2 == maximum):
        return [1, 2], pred2
    elif(pred3 == maximum):
        return [1, 3], pred3
    elif(pred4 == maximum):
        return [2, 1], pred4
    elif(pred5 == maximum):
        return [2, 2], pred5
    elif(pred6 == maximum):
        return [2, 3], pred6
    elif(pred7 == maximum):
        return [3, 1], pred7
    elif(pred8 == maximum):
        return [3, 2], pred8
    elif(pred9 == maximum):
        return [3, 3], pred9

def shift_drone_position(image_array, elevation, position_array):   
    # The RGB FOV is 69 deg x 42 deg (H X V)
    # 34.5 (69/2) is 34.5/180*pi
    # 21 (42/2) is 21/180*pi
    horizontal_shift = elevation * np.tan(34.5/180*np.pi)
    vertical_shift = elevation * np.tan(21/180*np.pi)

    rounded_horizontal = round(horizontal_shift)
    rounded_vertical = round(vertical_shift)
    
    if (position_array[0] == 1):
        #go left some predetermined distance
        p1.left_velocity(rounded_horizontal, 1)
    elif (position_array[0] == 3):
        #go right some predetermined distance
        p1.right_velocity(rounded_horizontal, 1)

    if (position_array[1] == 1):
        #go up some predetermined distance
        p1.forward_velocity(rounded_vertical, 1)
    elif (position_array[1] == 3):
        #go down some predetermined distance
        p1.backwards_velocity(rounded_vertical, 1)

    p1.downwards_velocity(5m)

coords_1 = (52.2296756, 21.0122287)
coords_2 = (52.406374, 16.9251681)

print(geopy.distance.geodesic(coords_1, coords_2).m) #Print out the distance in meters between two points

arm_and_takeoff(10)
condition_yaw(0)
Current_location_x= vehicle.location.global_relative_frame.lat
Current_location_y= vehicle.location.global_relative_frame.lon
Search_Pattern_Left()

'''
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
'''
print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()


