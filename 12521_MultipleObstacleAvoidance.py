# Import Statements #
import pyrealsense2 as rs
import cv2
import numpy as np
import modules # separate python file with custom functions - must be in same directory
import time

# Variables
frame_count = 0
drone_width = 0.01  # meters
drone_length = 0.1  # meters
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
clipping_distance_in_meters = 0.5# 1 meter ##############################################################################
clipping_distance = clipping_distance_in_meters / depth_scale
# Create an align object
align_to = rs.stream.color  # stream type
align = rs.align(align_to)  # perform alignment of depth frames to other frames

try:
    while True:
        frame_count += 1  # increment frame counter
        frames = pipeline.wait_for_frames()  # get frameset (color and depth)

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

        # --------------------------------------------------------------------------------------------------------------#

        # Canny Edge Identification
        new_img, thresh = modules.opencv_process(bg_removed)  # size - 240 x 320
        img_width, img_height = new_img.shape  # Image pixel dimensions

        # Is obstacle?
        contours, hierarchy = cv2.findContours(new_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnt = sorted(contours, key=cv2.contourArea)  # sort all the contours from smallest to largest

        # Check if obstacle is present
        if len(contours) != 0: # there is an edge found in the image
            is_obstacle = True
        else:
            is_obstacle = False # no obstacles found
            contour_img = new_img # display blank image

        if is_obstacle is True:  # obstacle is present
            start = time.time() # finding the execution time
            # Draw contours
            contour_img = cv2.drawContours(thresh, cnt[-1], 0, (0, 255, 0), -1)  # draw the largest contour
            # Get Variables
            cx1, cy1 = modules.pixel_coords(contour_img, cnt[-1])  # draw midpoint of largest contour
            aligned_depth_frame_dist = aligned_depth_frame.as_depth_frame()  # Depth frame to get distances

            if len(contours) > 1: # more than one obstacle
                # Draw contours
                contour_img = cv2.drawContours(thresh, cnt[-1], 0, (0, 255, 0), -1)  # draw the largest contour
                cx1, cy1 = modules.pixel_coords(contour_img, cnt[-1])  # draw midpoint of largest contour
                cx2, cy2 = modules.pixel_coords(contour_img, cnt[-2])  # draw midpoint of second largest contour

                # Required Values for Computations
                midx, midy = int(img_width / 2), int(img_height / 2)  # image midpoint pixel coordinates
                area1 = cv2.contourArea(cnt[-1]) # largest area (index = -1)
                area2 = cv2.contourArea(cnt[-2]) # second largest area (index = -2)

                # Begin Computations
                depth1 = aligned_depth_frame_dist.get_distance(cx1, cy1) # get distance to largest contour
                depth2 = aligned_depth_frame_dist.get_distance(cx2, cy2) # get distance to second largest contour
                depth_diff = abs(depth1 - depth2) # how far apart the obstacles are in depth

                # CASE 1 - TWO OBSTACLES AT SIMILAR DEPTH
                if depth_diff < drone_length: # if the difference in depth is not large enough for drone to fit between
                    if (area1 - area2) < 10:   # if the largest and second largest are similar in size
                        contour_img = cv2.drawContours(thresh, cnt[-2], 0, (0, 255, 0), -1) # draw second largest contour
                        dist_ab = modules.calculate_distance(aligned_frames, aligned_depth_frame_dist, cx1, cy1, cx2, cy2)  # Calculate euclidean distance between points

                        # CASE 1A - Drone fits in the gap
                        if drone_width < (dist_ab): # if the drone fits in the gap
                            mid_gapx, mid_gapy = int(abs(cx1+cx2)/2), int(abs(cy1+cy2)/2) # pixel coordinates of gap center
                            if mid_gapx < midx: # drone needs to move left
                                print("Move Left until drone is centered between gap.")
                                dist_to_move = (aligned_depth_frame_dist.get_distance(cx2, cy2)) + drone_length  # make sure to move past second largest obstacle
                                print("Move forward: ", dist_to_move, "meters.")
                            else:
                                print("Move Right until drone is centered between gap.")
                                # determine forward distance
                                dist_to_move = (aligned_depth_frame_dist.get_distance(cx2, cy2)) + drone_length  # make sure to move past second largest obstacle
                                print("Move forward: ", dist_to_move, "meters.")

                        # CASE 1B - Drone DOES NOT fit in the gap
                        else: # move around both obstacles
                            # Find min and max points of both contours
                            if max(cx1,cx2) < midx: # obstacle is on the left of the midpoint
                                dist_to_move = (aligned_depth_frame_dist.get_distance(cx2, cy2)) + drone_length  # make sure to move past second largest obstacle
                                if max(cx1, cx2) < int(img_width/6): # the obstacles are in the far left of the image
                                    print("Move forward: ", dist_to_move, "meters.")
                                else: # obstacle is in middle left of image
                                    print("Move Right")
                                    print("Move forward: ", dist_to_move, "meters.")
                                    # quantify forward movement
                            else: # rightmost obstacle is on the right of the midpoint
                                dist_to_move = (aligned_depth_frame_dist.get_distance(cx2, cy2)) + drone_length  # make sure to move past second largest obstacle
                                if min(cx1, cx2) > int(img_width* (5/6)): # the obstacles are in the far right of the image
                                    print("Move forward: ", dist_to_move, "meters.")
                                else: # the obstacles are in the middle right of the image
                                    print("Move Left.")
                                    print("Move forward: ", dist_to_move, "meters.")

                    # CASE 2: STAGGERED MOVEMENT
                    else:  # area difference is large enough to handle closest one first
                        # Step 1: Avoid First Obstacle
                        dist_to_move = (aligned_depth_frame_dist.get_distance(cx1, cy1)) + drone_length # distance to pass first obstacle
                        direction = modules.avoid_center_obstacle(cx1, img_width, frame_count) # returns the first direction
                        print("Move forward: ", dist_to_move, "meters.")
                        # Step 2: Avoid Second Obstacle
                        if direction == 1: # drone moved right the first time
                            print("Sharp left to original position.")
                        if direction == 2: # drone moved left the first time
                            print("Sharp right to original position.")
            # Print the total execution time for this process
            #print("time: ",time.time()-start)

            # Display Image
            cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
            cv2.imshow('Align Example', contour_img)
            key = cv2.waitKey(1)

            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27 or frame_count > 200:
                cv2.destroyAllWindows()
                break

finally:
    pipeline.stop() # stop the image stream
