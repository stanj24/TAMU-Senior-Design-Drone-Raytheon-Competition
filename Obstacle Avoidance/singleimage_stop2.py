# Import #
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
import geopy
from geopy.distance import geodesic

import pyrealsense2 as rs
import cv2
import time
import numpy as np
import math as m
import argparse
import datetime

# Argument Parser -> for communication with the vehicle #
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect', default='127.0.0.1:14551')
args = parser.parse_args()
connection_string = args.connect
## ESTABLISH CONNECTION ##
# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(args.connect, baud=57600, wait_ready=True)


## VEHICLE FUNCTIONS ##
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
    m_lat = (1 / ((2 * m.pi / 360) * earth_radius)) / 1000
    lat_new = lat + (lat_translation_meters * m_lat)

    # Calculate right, which is long_translation_meters right
    m_long = (1 / ((2 * m.pi / 360) * earth_radius)) / 1000  # 1 meter in degree
    long_new = long + (long_translation_meters * m_long) / m.cos(lat * (m.pi / 180))
    altitude = 0

    return long_new  # Changed


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
    m_lat = (1 / ((2 * m.pi / 360) * earth_radius)) / 1000
    lat_new = lat + (lat_translation_meters * m_lat)

    # Calculate right, which is long_translation_meters right
    m_long = (1 / ((2 * m.pi / 360) * earth_radius)) / 1000  # 1 meter in degree
    long_new = long + (long_translation_meters * m_long) / m.cos(lat * (m.pi / 180))
    altitude = 0

    return lat_new  # Changed


def condition_yaw(heading, relative=False):  # This function is to set the drone as straight
    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        1,  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


## MOVEMENT FUNCTIONS ##
def forward_GPS_point(rounded_vertical):  # New Going Forward Function - Jeremiah
    Current_location_x_new = vehicle.location.global_relative_frame.lat  # Getting the starting poitns
    Current_location_y_new = vehicle.location.global_relative_frame.lon
    New_straight_point = translate_up_down(Current_location_x_new, Current_location_y_new, rounded_vertical, 0)
    print("Flying Straight")
    Altitude = vehicle.location.global_relative_frame.alt
    Travel_point_2 = LocationGlobalRelative(New_straight_point, Current_location_y_new, Altitude)
    vehicle.simple_goto(Travel_point_2)
    time.sleep(10)


def backwards_GPS_point(rounded_vertical):  # New Going Function - Jeremiah
    Current_location_x_new = vehicle.location.global_relative_frame.lat  # Getting the starting poitns
    Current_location_y_new = vehicle.location.global_relative_frame.lon
    New_straight_point = translate_up_down(Current_location_x_new, Current_location_y_new, -1 * rounded_vertical, 0)
    print("Flying Backwards")
    Altitude = vehicle.location.global_relative_frame.alt
    Travel_point_2 = LocationGlobalRelative(New_straight_point, Current_location_y_new, Altitude)
    vehicle.simple_goto(Travel_point_2)
    time.sleep(10)


def left_GPS_point(rounded_horizontal):  # New Going Function - Jeremiah
    Current_location_x_new = vehicle.location.global_relative_frame.lat  # Getting the starting poitns
    Current_location_y_new = vehicle.location.global_relative_frame.lon
    Longitude_new = translate_latlong(Current_location_x_new, Current_location_y_new, 0, -1 * rounded_horizontal)
    print("Flying left")
    Altitude = vehicle.location.global_relative_frame.alt
    Travel_point = LocationGlobalRelative(Current_location_x_new, Longitude_new, Altitude)
    vehicle.simple_goto(Travel_point)
    time.sleep(10)


def right_GPS_point(rounded_horizontal):  # New Going Function - Jeremiah
    Current_location_x_new = vehicle.location.global_relative_frame.lat  # Getting the starting poitns
    Current_location_y_new = vehicle.location.global_relative_frame.lon
    Longitude_new = translate_latlong(Current_location_x_new, Current_location_y_new, 0, rounded_horizontal)
    print("Flying right")
    Altitude = vehicle.location.global_relative_frame.alt
    Travel_point = LocationGlobalRelative(Current_location_x_new, Longitude_new, Altitude)
    vehicle.simple_goto(Travel_point)
    time.sleep(10)


######################################################
##  Depth parameters - reconfigurable               ##
######################################################

# Sensor-specific parameter, for D435: https://www.intelrealsense.com/depth-camera-d435/
STREAM_TYPE = [rs.stream.depth, rs.stream.color]  # rs2_stream is a types of data provided by RealSense device
FORMAT = [rs.format.z16, rs.format.bgr8]  # rs2_format is identifies how binary data is encoded within a frame
DEPTH_WIDTH = 640  # Defines the number of columns for each frame or zero for auto resolve
DEPTH_HEIGHT = 480  # Defines the number of lines for each frame or zero for auto resolve
COLOR_WIDTH = 640
COLOR_HEIGHT = 480
FPS = 30
DEPTH_RANGE_M = [0.1, 8.0]  # Replace with your sensor's specifics, in meter

obstacle_line_height_ratio = 0.3 # [0-1]: 0-Top, 1-Bottom. The height of the horizontal line to find distance to obstacle.
obstacle_line_thickness_pixel = 10  # [1-DEPTH_HEIGHT]: Number of pixel rows to use to generate the obstacle distance message. For each column, the scan will return the minimum value for those pixels centered vertically in the image.

# List of filters to be applied, in this order.
# https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md

filters = [
    [True, "Decimation Filter", rs.decimation_filter()],
    [True, "Threshold Filter", rs.threshold_filter()],
    [True, "Depth to Disparity", rs.disparity_transform(True)],
    [True, "Spatial Filter", rs.spatial_filter()],
    [True, "Temporal Filter", rs.temporal_filter()],
    [False, "Hole Filling Filter", rs.hole_filling_filter()],
    [True, "Disparity to Depth", rs.disparity_transform(False)]
]

#
# The filters can be tuned with opencv_depth_filtering.py script, and save the default values to here
# Individual filters have different options so one have to apply the values accordingly
#

# decimation_magnitude = 8
# filters[0][2].set_option(rs.option.filter_magnitude, decimation_magnitude)

threshold_min_m = DEPTH_RANGE_M[0]
threshold_max_m = DEPTH_RANGE_M[1]
if filters[1][0] is True:
    filters[1][2].set_option(rs.option.min_distance, threshold_min_m)
    filters[1][2].set_option(rs.option.max_distance, threshold_max_m)

debug_enable = 0  # add an option if debug_enable = 1 to display the images

# default exit code is failure - a graceful termination with a
# terminate signal is possible.
exit_code = 1

######################################################
##  Global variables                                ##
######################################################

# Use this to rotate all processed data
camera_facing_angle_degree = 0
device_id = None

# Camera-related variables
pipe = None
depth_scale = 0
colorizer = rs.colorizer()
depth_hfov_deg = 85
depth_vfov_deg = 57

# The name of the display window
display_name = 'Input/output depth'
rtsp_streaming_img = None

# Data variables
vehicle_pitch_rad = 0

# Obstacle distances in front of the sensor, starting from the left in increment degrees to the right
# See here: https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE
min_depth_cm = int(DEPTH_RANGE_M[0] * 100)  # In cm
max_depth_cm = int(DEPTH_RANGE_M[1] * 100)  # In cm, should be a little conservative
distances_array_length = 72
angle_offset = None
increment_f = None
distances = np.ones((distances_array_length,), dtype=np.uint16) * (max_depth_cm + 1)

if debug_enable == 1:
    print("INFO: Debugging option enabled")
    cv2.namedWindow(display_name, cv2.WINDOW_AUTOSIZE)
else:
    print("INFO: Debugging option DISABLED")


# Functions #

######################################################
##  Functions - D4xx cameras                        ##
######################################################

# Establish connection to the Realsense camera
def realsense_connect():
    global pipe, depth_scale
    # Declare RealSense pipe, encapsulating the actual device and sensors
    pipe = rs.pipeline()

    # Configure image stream(s)
    config = rs.config()
    if device_id:
        # connect to a specific device ID
        config.enable_device(device_id)
    config.enable_stream(STREAM_TYPE[0], DEPTH_WIDTH, DEPTH_HEIGHT, FORMAT[0], FPS)
    # if RTSP_STREAMING_ENABLE is True:
    config.enable_stream(STREAM_TYPE[1], COLOR_WIDTH, COLOR_HEIGHT, FORMAT[1], FPS)

    # Start streaming with requested config
    profile = pipe.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("INFO: Depth scale is: %s" % depth_scale)


########################################################################################################################

### PROCESS FUNCTIONS ##################################################################################################

def take_image3():  # pipeline must be started before calling
    time.sleep(1)
    # get color frame
    frames = pipe.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())

    depth_image = np.asanyarray(depth_frame.get_data())

    return depth_frame, color_frame, depth_image, color_image


def apply_filters(frame, DEPTH_RANGE_M):
    ''' Apply post-processing filters to depth image to fill holes'''
    filters = [
        [True, "Decimation Filter", rs.decimation_filter()],
        [True, "Threshold Filter", rs.threshold_filter()],
        [True, "Depth to Disparity", rs.disparity_transform(True)],
        [True, "Spatial Filter", rs.spatial_filter()],
        [True, "Temporal Filter", rs.temporal_filter()],
        [False, "Hole Filling Filter", rs.hole_filling_filter()],
        [True, "Disparity to Depth", rs.disparity_transform(False)]
    ]

    # Decimation Filter
    threshold_min_m = DEPTH_RANGE_M[0]
    threshold_max_m = DEPTH_RANGE_M[1]
    if filters[1][0] is True:
        filters[1][2].set_option(rs.option.min_distance, threshold_min_m)
        filters[1][2].set_option(rs.option.max_distance, threshold_max_m)

    # Other Filters
    filtered_frame = depth_frame
    for i in range(len(filters)):
        if filters[i][0] is True:
            filtered_frame = filters[i][2].process(filtered_frame)

    return filtered_frame


def frame_to_img(frame):
    '''Convert images to numpy arrays'''

    # Extract depth in matrix form
    depth_data = frame.as_frame().get_data()
    depth_matrix = np.asanyarray(depth_data)

    return depth_matrix


# Obstacle Avoiding Functions #

# Setting parameters for the OBSTACLE_DISTANCE message based on actual camera's intrinsics and user-defined params
def set_obstacle_distance_params():
    global angle_offset, camera_facing_angle_degree, increment_f, depth_scale, depth_hfov_deg, depth_vfov_deg, obstacle_line_height_ratio, obstacle_line_thickness_pixel

    # Obtain the intrinsics from the camera itself
    profiles = pipe.get_active_profile()
    depth_intrinsics = profiles.get_stream(STREAM_TYPE[0]).as_video_stream_profile().intrinsics
    print("INFO: Depth camera intrinsics: %s" % depth_intrinsics)

    # For forward facing camera with a horizontal wide view:
    #   HFOV=2*atan[w/(2.fx)],
    #   VFOV=2*atan[h/(2.fy)],
    #   DFOV=2*atan(Diag/2*f),
    #   Diag=sqrt(w^2 + h^2)
    depth_hfov_deg = m.degrees(2 * m.atan(DEPTH_WIDTH / (2 * depth_intrinsics.fx)))
    depth_vfov_deg = m.degrees(2 * m.atan(DEPTH_HEIGHT / (2 * depth_intrinsics.fy)))
    print("INFO: Depth camera HFOV: %0.2f degrees" % depth_hfov_deg)
    print("INFO: Depth camera VFOV: %0.2f degrees" % depth_vfov_deg)

    angle_offset = camera_facing_angle_degree - (depth_hfov_deg / 2)
    increment_f = depth_hfov_deg / distances_array_length
    print("INFO: OBSTACLE_DISTANCE angle_offset: %0.3f" % angle_offset)
    print("INFO: OBSTACLE_DISTANCE increment_f: %0.3f" % increment_f)
    print("INFO: OBSTACLE_DISTANCE coverage: from %0.3f to %0.3f degrees" %
          (angle_offset, angle_offset + increment_f * distances_array_length))

    # Sanity check for depth configuration
    if obstacle_line_height_ratio < 0 or obstacle_line_height_ratio > 1:
        print("Please make sure the horizontal position is within [0-1]: %s" % obstacle_line_height_ratio)
        sys.exit()

    if obstacle_line_thickness_pixel < 1 or obstacle_line_thickness_pixel > DEPTH_HEIGHT:
        print("Please make sure the thickness is within [0-DEPTH_HEIGHT]: %s" % obstacle_line_thickness_pixel)
        sys.exit()


# Find the height of the horizontal line to calculate the obstacle distances
#   - Basis: depth camera's vertical FOV, user's input
#   - Compensation: vehicle's current pitch angle
def find_obstacle_line_height():
    global vehicle_pitch_rad, depth_vfov_deg, DEPTH_HEIGHT

    # Basic position
    obstacle_line_height = DEPTH_HEIGHT * obstacle_line_height_ratio

    # Compensate for the vehicle's pitch angle if data is available
    if vehicle_pitch_rad is not None and depth_vfov_deg is not None:
        delta_height = m.sin(vehicle_pitch_rad / 2) / m.sin(m.radians(depth_vfov_deg) / 2) * DEPTH_HEIGHT
        obstacle_line_height += delta_height

    # Sanity check
    if obstacle_line_height < 0:
        obstacle_line_height = 0
    elif obstacle_line_height > DEPTH_HEIGHT:
        obstacle_line_height = DEPTH_HEIGHT

    return obstacle_line_height


# Calculate the distances array by dividing the FOV (horizontal) into $distances_array_length rays,
# then pick out the depth value at the pixel corresponding to each ray. Based on the definition of
# the MAVLink messages, the invalid distance value (below MIN/above MAX) will be replaced with MAX+1.
#
# [0]    [35]   [71]    <- Output: distances[72]
#  |      |      |      <- step = width / 72
#  ---------------      <- horizontal line, or height/2
#  \      |      /
#   \     |     /
#    \    |    /
#     \   |   /
#      \  |  /
#       \ | /
#       Camera          <- Input: depth_mat, obtained from depth image
#
# Note that we assume the input depth_mat is already processed by at least hole-filling filter.
# Otherwise, the output array might not be stable from frame to frame.
# @njit   # Uncomment to optimize for performance. This uses numba which requires llmvlite (see instruction at the top)
def distances_from_depth_image(obstacle_line_height, depth_mat, distances, min_depth_m, max_depth_m,
                               obstacle_line_thickness_pixel):
    # Parameters for depth image
    depth_img_width = depth_mat.shape[1]
    depth_img_height = depth_mat.shape[0]

    # Parameters for obstacle distance message
    step = depth_img_width / distances_array_length

    for i in range(distances_array_length):
        # Each range (left to right) is found from a set of rows within a column
        #  [ ] -> ignored
        #  [x] -> center + obstacle_line_thickness_pixel / 2
        #  [x] -> center = obstacle_line_height (moving up and down according to the vehicle's pitch angle)
        #  [x] -> center - obstacle_line_thickness_pixel / 2
        #  [ ] -> ignored
        #   ^ One of [distances_array_length] number of columns, from left to right in the image
        center_pixel = obstacle_line_height
        upper_pixel = center_pixel + obstacle_line_thickness_pixel / 2
        lower_pixel = center_pixel - obstacle_line_thickness_pixel / 2

        # Sanity checks
        if upper_pixel > depth_img_height:
            upper_pixel = depth_img_height
        elif upper_pixel < 1:
            upper_pixel = 1
        if lower_pixel > depth_img_height:
            lower_pixel = depth_img_height - 1
        elif lower_pixel < 0:
            lower_pixel = 0

        # Converting depth from uint16_t unit to metric unit. depth_scale is usually 1mm following ROS convention.
        # dist_m = depth_mat[int(obstacle_line_height), int(i * step)] * depth_scale
        min_point_in_scan = np.min(depth_mat[int(lower_pixel):int(upper_pixel), int(i * step)])
        dist_m = min_point_in_scan * depth_scale

        # Default value, unless overwritten:
        #   A value of max_distance + 1 (cm) means no obstacle is present.
        #   A value of UINT16_MAX (65535) for unknown/not used.
        distances[i] = 65535

        # Note that dist_m is in meter, while distances[] is in cm.
        if dist_m > min_depth_m and dist_m < max_depth_m:
            distances[i] = dist_m * 100
    return distances


def check_obstacles(dist_array):
    min_dist = min(dist_array)
    #min_dist = np.min(dist_array[np.nonzero(dist_array)])
    # is obstacle present?
    if min_dist < 200:  # distance to obstacle is less than 2 m away
        print("Obstacle found at distance: ",min_dist," cm")
        print("OBSTACLE SEEN: LAND")
        vehicle.mode = VehicleMode("LAND")  # Land the drone immediately
        vehicle.close()
    else:
        print("NO OBSTACLE: MOVE BACK")
        print("closest obstacle: ", min_dist)
        backwards_GPS_point(5)  # if obstacle is NOT recognized -> move backwards
        vehicle.mode = VehicleMode("LAND")  # Land the drone
        vehicle.close()


# Begin of the main loop
# start the camera connection
count = 0
while count <= 0:
    realsense_connect()
    time.sleep(3)  # give the camera frames a chance to warmup
    arm_and_takeoff(2)  # takeoff to altitude of 5 feet = 1.5 meters
    condition_yaw(0)  # position drone straight

    # move forward 5 feet
    forward_GPS_point(5)

    # take image
    # depth_frame, color_frame, filtered_frame, depth_mat, color_image = take_image()
    depth_frame, color_frame, depth_image, color_image = take_image3()

    # save images
    ct = datetime.datetime.now()

    depth_im = np.asanyarray(colorizer.colorize(depth_frame).get_data())
    color_im = np.asanyarray(colorizer.colorize(color_frame).get_data())

    cv2.imwrite("depthim_" + str(ct) + ".jpg", depth_im)
    cv2.imwrite("colorim_" + str(ct) + ".jpg", color_im)

    # Create obstacle distance data from depth image
    obstacle_line_height = find_obstacle_line_height()
    dist_array = distances_from_depth_image(obstacle_line_height, depth_image, distances, DEPTH_RANGE_M[0], DEPTH_RANGE_M[1],
                                            obstacle_line_thickness_pixel)

    # check for obstacle and do appropriate movement
    check_obstacles(dist_array)

    count += 1


# stop the camera
pipe.stop()

if debug_enable == 1:
    # Prepare the data
    input_image = np.asanyarray(colorizer.colorize(depth_frame).get_data())
    output_image = np.asanyarray(colorizer.colorize(depth_frame).get_data())

    # Draw a horizontal line to visualize the obstacles' line
    x1, y1 = int(0), int(obstacle_line_height)
    x2, y2 = int(DEPTH_WIDTH), int(obstacle_line_height)
    line_thickness = obstacle_line_thickness_pixel
    cv2.line(output_image, (x1, y1), (x2, y2), (0, 255, 0), thickness=line_thickness)
    display_image = np.hstack((input_image, cv2.resize(output_image, (DEPTH_WIDTH, DEPTH_HEIGHT))))

    # Put the fps in the corner of the image
    #processing_speed = 1 / (time.time() - last_time)
    #text = ("%0.2f" % (processing_speed,)) + ' fps'
    #textsize = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
    #cv2.putText(display_image,
                #text,
                #org=(int((display_image.shape[1] - textsize[0] / 2)), int((textsize[1]) / 2)),
                #fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                #fontScale=0.5,
                #thickness=1,
                #color=(255, 255, 255))

    # Show the images
    cv2.imshow(display_name, display_image)
    cv2.waitKey(0)

    # Print all the distances in a line
    # print("%s" % (str(distances)))

    last_time = time.time()
