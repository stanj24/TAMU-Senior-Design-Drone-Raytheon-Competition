# this code is meant to show that the logo is recognized and the drone performs an action accordingly






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
import datetime
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
    Travel_point_2 = LocationGlobalRelative(New_straight_point, Current_location_y_new, 7)
    vehicle.simple_goto(Travel_point_2)
    time.sleep(10)

def backwards_GPS_point(rounded_vertical): #New Going Function - Jeremiah 
    Current_location_x_new = vehicle.location.global_relative_frame.lat  # Getting the starting poitns
    Current_location_y_new = vehicle.location.global_relative_frame.lon
    New_straight_point = translate_up_down(Current_location_x_new, Current_location_y_new, -1*rounded_vertical, 0)
    print("Flying Backwards")
    Altitude = vehicle.location.global_relative_frame.alt
    Travel_point_2 = LocationGlobalRelative(New_straight_point, Current_location_y_new, 5)
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

# main code
img_og = cv2.imread("TAM-LogoBox.jpg")
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 8)
# Start streaming
pipeline.start(config)

#drone starts flying to 7 meters
arm_and_takeoff(7)
#drone repositions itself north
condition_yaw(0)

#take an initial image in the air after rising
a = take_image()
ct1 = datetime.datetime.now()
cv2.imwrite("initial_image_" + str(ct1) + ".jpg", a)

#compare logo.jpg and the picture taken on the camera
ev1 = evaluate_image(img_og, a)

with open('non_logo_validation_data.txt', 'a') as f:
    f.write("Recorded height is " + str(vehicle.location.global_relative_frame.alt))
    f.write("\n")
    f.write(str(ev1) + " pixels matching initially")
    f.write("\n")
    f.write("\n")
print(str(ev1) + " pixels matching initially")

#move forward 8 meters
forward_GPS_point(8)
#take a new image
b = take_image()
ct2 = datetime.datetime.now()

#compare logo.jpg and the picture taken on the camera
ev2 = evaluate_image(img_og, b)
print(str(ev2) + " pixels matching after moving forward")

#if the number of matching pixels is above 15, then logo is identified
if (ev2 > 15):
    cv2.imwrite("logo_image_success_first_time_" + str(ct2) + ".jpg", b)
    with open('non_logo_validation_data.txt', 'a') as f:
        f.write("Recorded height is " + str(vehicle.location.global_relative_frame.alt))
        f.write("\n")
        f.write(str(ev2) + " pixels matching after moving forward to look for logo(found)")
        f.write("\n")
        f.write("\n")
        f.write("\n")
    #go ahead and land
    print("This meets the threshold, time to land")
    vehicle.mode = VehicleMode("LAND")
    vehicle.close()
#if the number of matching pixels is <= 15, then logo is not identified
else:
    cv2.imwrite("logo_image_failed_first_time_" + str(ct2) + ".jpg", b)
    with open('non_logo_validation_data.txt', 'a') as f:
        f.write("Recorded height is " + str(vehicle.location.global_relative_frame.alt))
        f.write("\n")
        f.write(str(ev2) + " pixels matching after moving forward to look for logo(not found)")
        f.write("\n")
        f.write("\n")
    #move forward 8 meters
    forward_GPS_point(8)
    #take a new image
    c = take_image()
    ct3 = datetime.datetime.now()

    #compare logo.jpg and the picture taken on the camera
    ev3 = evaluate_image(img_og, c)
    print(str(ev3) + " pixels matching after moving forward")

    #if the number of matching pixels is above 15, then logo is identified
    if (ev3 > 15):
        cv2.imwrite("logo_on_second_round_sucxess_" + str(ct3) + ".jpg", c)
        with open('non_logo_validation_data.txt', 'a') as f:
            f.write("Recorded height is " + str(vehicle.location.global_relative_frame.alt))
            f.write("\n")
            f.write(str(ev3) + " pixels matching after moving forward to look for logo again(found)")
            f.write("\n")
            f.write("\n")
            f.write("\n")
        print("This meets the threshold, time to land")
        vehicle.mode = VehicleMode("LAND")
        vehicle.close()
    #if the number of matching pixels is <= 15, then logo is not identified
    else:
        cv2.imwrite("logo_on_second_round_failed_" + str(ct3) + ".jpg", c)
        with open('non_logo_validation_data.txt', 'a') as f:
            f.write("Recorded height is " + str(vehicle.location.global_relative_frame.alt))
            f.write("\n")
            f.write(str(ev3) + " pixels matching after moving forward to look for logo again(not found)")
            f.write("\n")
            f.write("\n")
            f.write("\n")
        #move forward 8 meters
        forward_GPS_point(8)

        #at the end of the script, so go ahead and land
        print("This does NOT meets the threshold, time to land")
        vehicle.mode = VehicleMode("LAND")
        vehicle.close()

pipeline.stop()
