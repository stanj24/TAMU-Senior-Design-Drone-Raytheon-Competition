# Import Statements #
import pyrealsense2 as rs
import cv2
import numpy as np
import modules # separate python file with custom functions - must be in same directory
import time

# Variables
frame_count = 0
drone_width = 0.6 # meters
drone_length = 0.6 # meters
is_obstacle = bool

## SETTING UP REALSENSE CAMERA ##
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)
queue = rs.frame_queue(50, keep_frames=True)

## GETTING CAMERA INFORMATION ##
# Get depth scale
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
# We will be removing the background of objects more than clipping_distance_in_meters meters away
clipping_distance_in_meters = 0.5 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale
# Create an align object
align_to = rs.stream.color # stream type
align = rs.align(align_to) # perform alignment of depth frames to other frames

try:
    while True:
        frame_count += 1 # increment frame counter
        frames = pipeline.wait_for_frames() # get frameset (color and depth)

        ## DEPTH FRAME POST PROCESSING ##
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()
        # Apply post processing to depth frame
        aligned_depth_frame = modules.post_process(aligned_depth_frame)

        ## GET BACKGROUND REMOVED ##
        bg_removed = modules.no_background(aligned_depth_frame, color_frame, clipping_distance)

        #--------------------------------------------------------------------------------------------------------------#

        # Canny Edge Identification
        new_img, thresh = modules.opencv_process(bg_removed) # size - 240 x 320
        img_width, img_height = new_img.shape # Image pixel dimensions

        # Is obstacle present?
        contours, hierarchy = cv2.findContours(new_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # find contours in image
        cnt = sorted(contours, key=cv2.contourArea)  # sort all the contours from smallest to largest
        if len(contours) != 0: # there is an edge found in the image
            is_obstacle = True
        else:
            is_obstacle = False # no obstacles found
            contour_img = new_img # display the blank image

        if is_obstacle is True: # obstacle is present
            # Draw contours
            contour_img = cv2.drawContours(thresh, cnt[-1], 0, (0, 255, 0), -1)  # draw the largest contour
            # Get variables
            cx1, cy1 = modules.pixel_coords(contour_img, cnt[-1])  # draw midpoint of largest contour
            aligned_depth_frame_dist = aligned_depth_frame.as_depth_frame() # convert the depth frame to get distances
            # Determine movements
            if cx1 < int(img_width/6) or cx1 > int(img_width*(7/8)):  # obstacle is close to the edge of image
                # Get depth to that obstacle
                dist_to_move = (aligned_depth_frame_dist.get_distance(cx1, cy1)) + drone_length
                print(frame_count, "- Move forward: ", dist_to_move, "meters.")

            else: # obstacle is near center of image
                modules.avoid_center_obstacle(cx1, img_width, frame_count)

            # Display Image
            cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
            cv2.imshow('Align Example', contour_img)
            key = cv2.waitKey(1)

            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
        else:
            print("No obstacle - continue straight.")

finally:
    pipeline.stop() # stop the image stream