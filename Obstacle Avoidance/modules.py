# Import Statements #
import pyrealsense2 as rs
import cv2
import numpy as np
import time
import math

def post_process(depth_frame):
    ''' Makes the depth images more defined and reduces disparities. '''

    # Post Processing Filters - applied to depth frame after alignment
    decimation = rs.decimation_filter()
    spatial = rs.spatial_filter()
    temporal = rs.temporal_filter()
    hole_filling = rs.hole_filling_filter()
    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)

    # Apply post processing
    frame = decimation.process(depth_frame)
    frame = depth_to_disparity.process(frame)
    frame = spatial.process(frame)
    frame = temporal.process(frame)
    frame = disparity_to_depth.process(frame)
    new_depth_frame = hole_filling.process(frame)

    return new_depth_frame

def no_background(aligned_depth_frame, color_frame, clipping_distance):
    ''' Grays pixels in an image past clipping distance.'''

    # get the data for depth and color information of the image as arrays
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    # get the map dimensions
    depth_colormap_dim = depth_colormap.shape
    color_colormap_dim = color_image.shape

    # resize color image to match depth image for display if they vary
    if depth_colormap_dim != color_colormap_dim:
        color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                 interpolation=cv2.INTER_AREA)

    # remove background - Set pixels further than clipping_distance to grey
    grey_color = 153
    depth_image_3d = np.dstack(
        (depth_image, depth_image, depth_image))  # depth image is 1 channel, color is 3 channels
    bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

    return bg_removed


def opencv_process(image):
    ''' Find the edges of the objects in the image. '''

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #grayscale
    median = cv2.medianBlur(gray, 5) # median blur
    gaussian = cv2.GaussianBlur(median, (7, 7), 0) # gaussian blur
    ret, thresh = cv2.threshold(gaussian, 127, 255, cv2.THRESH_BINARY_INV)  # object is white, background is black
    edges = cv2.Canny(thresh, 25, 100)  # 25 50 # canny edge identification
    return edges, thresh

def pixel_coords(image, max_contour):
    ''' Find object center points. '''

    global cx, cy
    # find 3d coordinate of obstacle center point
    x, y, w, h = cv2.boundingRect(max_contour)
    cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2) # draw rectangle around max contour
    M = cv2.moments(max_contour) # find the center point of max contour using moments
    if M['m00'] != 0:
        cx = int(M['m10'] / M['m00']) # obstacle x center
        cy = int(M['m01'] / M['m00']) # obstacle y center
        cv2.circle(image, (cx, cy), 7, (255, 0, 0), -1) # put a circle at the center point
        cv2.putText(image, "center", (cx - 20, cy - 20), # write "center" at the center point
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    return cx, cy

def calculate_distance(self,depth_frame, ix, iy, x, y):
    ''' Find the euclidean distance between two pixel locations. '''

    # get stream information
    prof = self.get_profile().as_video_stream_profile()
    intr = prof.get_intrinsics()

    # get distance from camera to each point
    udist = depth_frame.get_distance(ix,iy)
    vdist = depth_frame.get_distance(x, y)

    # convert the pixel to a 3D point based on video intrinsics
    point1 = rs.rs2_deproject_pixel_to_point(intr, [ix, iy], udist)
    point2 = rs.rs2_deproject_pixel_to_point(intr, [x, y], vdist)

    # calculate the euclidean distance
    dist = math.sqrt(
        math.pow(point1[0] - point2[0], 2) + math.pow(point1[1] - point2[1],2) + math.pow(
            point1[2] - point2[2], 2))

    return dist

def avoid_center_obstacle(cx, img_width, frame_count):
    ''' Which direction to move around each obstacle. '''

    if cx < int(img_width/2):  # object is on the left of the center
        print(frame_count, "- Move Right.") # drone will keep moving right until otherwise indicated
        direction = 1 # right = 1
    elif cx > int(img_width/2):  # object is on right of the center
        print(frame_count, "-Move Left.") # drone keeps moving left till otherwise indicated
        direction = 2 # left = 2
    else:
        direction = 0
    return direction # if obstacle is staggered, this ensures that the second direction is opposite of the first
