import numpy as np
import cv2


# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = ((img[:, :, 0] > rgb_thresh[0]) & (img[:, :, 1] > rgb_thresh[1])
                     & (img[:, :, 2] > rgb_thresh[2]))
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select


# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles


# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = np.deg2rad(yaw)

    # Apply a rotation
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)
    # Return the result  
    return xpix_rotated, ypix_rotated


# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world


# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))  # keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # NOTE: camera image is coming to you in Rover.img
    img = Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                              [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                              [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                              [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                              ])
    # 2) Apply perspective transform
    warped = perspect_transform(img, source, destination)
    # mask far away pixels as they are more distorted
    distance = 120
    height, width, depth = warped.shape
    circle_img = np.zeros((height, width), np.uint8)
    cv2.circle(circle_img, (width // 2, height), distance, 1, thickness=-1)
    masked_data = cv2.bitwise_and(warped, warped, mask=circle_img)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    rgb_thresh = (160, 160, 160)
    nav_select = cv2.inRange(masked_data, rgb_thresh, (255, 255, 255))
    # threshold for rock samples
    hsv_low_thresh = (20, 100, 100)
    hsv_high_thresh = (30, 255, 255)
    hsv_img = cv2.cvtColor(masked_data, cv2.COLOR_RGB2HSV)
    sample_select = cv2.inRange(hsv_img, hsv_low_thresh, hsv_high_thresh)
    # Anything not navigable or sample is an obstacle
    obstacle_select = cv2.bitwise_not(cv2.bitwise_or(nav_select, sample_select))

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:, :, 0] = obstacle_select
    Rover.vision_image[:, :, 1] = sample_select
    Rover.vision_image[:, :, 2] = nav_select
    # 5) Convert map image pixel values to rover-centric coords
    nav_rcoord_x, nav_rcoord_y = rover_coords(nav_select)
    sample_rcoord_x, sample_rcoord_y = rover_coords(sample_select)
    obstacle_rcoord_x, obstacle_rcoord_y = rover_coords(obstacle_select)
    # 6) Convert rover-centric pixel values to world coordinates
    world_size = 200
    scale = 10  # map scale
    obstacle_world_coord = pix_to_world(obstacle_rcoord_x, obstacle_rcoord_y,
                                        Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
    sample_world_coord = pix_to_world(sample_rcoord_x, sample_rcoord_y,
                                      Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
    nav_world_coord = pix_to_world(nav_rcoord_x, nav_rcoord_y,
                                   Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    # We keep adding each time a pixel is detected as navigable or sample
    # We need to setup a limit for the value of the channel to avoid resetting it.
    # In this way, our pixel intensity represents a kind of certainty in the classification of that point.
    # Update map only if rover is not pitching too much
    pitching_limit = 1
    if (Rover.pitch < pitching_limit) or (Rover.pitch > 360 - pitching_limit):
        # set limit for map pixel values
        upper_limit = np.full_like(Rover.worldmap[:, :, 0], 255)
        # update obstacles
        updated = Rover.worldmap[:, :, 0]
        updated[obstacle_world_coord[1], obstacle_world_coord[0]] += 20
        Rover.worldmap[:, :, 0] = np.minimum(updated, upper_limit)
        # update navigable terrain
        updated = Rover.worldmap[:, :, 2]
        updated[nav_world_coord[1], nav_world_coord[0]] += 20
        Rover.worldmap[:, :, 2] = np.minimum(updated, upper_limit)
        # update rock samples
        updated = Rover.worldmap[:, :, 1]
        updated[sample_world_coord[1], sample_world_coord[0]] += 20
        Rover.worldmap[:, :, 1] = np.minimum(updated, upper_limit)
    # let's keep a record of places visited
    mark_size = 4  # size in pixels in the world map assumed as visited
    x_pos = Rover.pos[0]
    y_pos = Rover.pos[1]
    cv2.circle(Rover.visited, (int(np.round(x_pos)), int(np.round(y_pos))), mark_size, (0, 1, 0), 1)
    # Next line just for visual debugging
    #Rover.worldmap[:, :, 2] = Rover.visited[:, :, 1]

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    dist, angles = to_polar_coords(nav_rcoord_x, nav_rcoord_y)
    Rover.nav_dists = dist
    Rover.nav_angles = angles

    # Detect if we are in sight of sample
    # percentage of pixel in image to consider a as sample detected
    sample_threshold = 0.008
    if (np.sum(sample_select)) > (sample_threshold * sample_select.shape[0] * sample_select.shape[1]):
        dist, angles = to_polar_coords(sample_rcoord_x, sample_rcoord_y)
        Rover.sample_bearing = np.mean(angles)
        Rover.sample_dist = np.mean(dist)
        print('sample detected, bearing:', Rover.sample_bearing, ', distance', Rover.sample_dist)
    else:
        Rover.sample_bearing = None
        Rover.sample_dist = None
    # Check is rover is stuck
    if Rover.vel < 0.1:
        Rover.speed_check += 1
    elif Rover.vel > 0.5:
        # reset counter if rover is moving. Some hysteresis introduced to work better
        Rover.speed_check = 0
    return Rover
