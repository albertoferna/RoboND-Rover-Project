import numpy as np


# This is hacky, I included this here and in perception. Would need rework
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

# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle)
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Inverse transformation from the one defined in perception
# It's needed to get information from visited map
def world_to_pix(x_pix_world, y_pix_world, xpos, ypos, yaw, world_size, scale):
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(x_pix_world, y_pix_world, -xpos, -ypos, scale)
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix_tran, ypix_tran, -yaw)
    return xpix_rot, ypix_rot

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Check if we have vision data to make decisions with
    print(Rover.speed_check)
    print(Rover.mode)
    if Rover.nav_angles is not None:
        # For exploration we do not need to get close to non-navigable terrain
        # with this value we control the amount of navigable terrain that we would accept to move forward
        Rover.stop_forward = 300
        # maximum speed for rover to be proportional to nav terrain
        speed_prop = 10
        Rover.max_vel = min(2, len(Rover.nav_angles) / (Rover.stop_forward * speed_prop))
        Rover.max_vel = max(Rover.max_vel, 0.6)
        # check navigable terrain just in front of rover for close navigation
        distance = 40
        selected_angles = np.where(Rover.nav_dists < distance)
        navigable = Rover.nav_angles[selected_angles]
        trim_angle = 5
        nav_angles = navigable[np.where(np.abs(navigable) < (trim_angle * np.pi / 180))]
        if Rover.mode == 'forward':
            # Being stuck takes preference
            # Rover in forward mode and not moving for 50 frames
            if Rover.speed_check > 50:
                # Rover stuck. Stop to get out of situation
                print('I am stuck')
                Rover.mode = 'stop'
                Rover.brake = Rover.brake_set
                Rover.throttle = 0
            elif len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                # We are going to correct the mean navigation angle to help with visiting the whole map
                x_visited, y_visited = Rover.visited[:, :, 1].nonzero()
                # Convert world coordinates to local rover coordinates
                x_pos = Rover.pos[0]
                y_pos = Rover.pos[1]
                yaw = Rover.yaw
                angles = Rover.nav_angles
                world_size = Rover.worldmap.shape[0]
                bottom_offset = 6
                rover_visited = world_to_pix(y_visited, x_visited, x_pos, y_pos, yaw, world_size, 1)
                penalty_points_x = rover_visited[0][rover_visited[0] > bottom_offset]
                penalty_points_y = rover_visited[1][rover_visited[0] > bottom_offset]
                dist_penalty, angles_penalty = to_polar_coords(penalty_points_x, penalty_points_y)
                # check that there are at least one visited point
                if len(angles_penalty) > 0:
                    # Navigable terrain is considered twice to help with going over the same terrain more than once
                    angles_penalty_mean = angles.mean() - angles_penalty.mean() / 2
                else:
                    angles_penalty_mean = angles.mean()
                wall_bias = 0.5
                Rover.steer = np.clip((angles_penalty_mean * 90/np.pi) + wall_bias, -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                # we take the amount of frames that we have been stopped. We do a maneuver for other 50 frames
                if Rover.speed_check > 50 and not Rover.near_sample:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    # Turn in most likely direction
                    Rover.steer = 15
                    if Rover.speed_check > 100:
                        # try to get out
                        Rover.speed_check = 0
                # If we're stopped but see sufficient navigable terrain in front then go!
                elif len(nav_angles) * 10 >= Rover.go_forward and not Rover.near_sample:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle of local angles
                    Rover.steer = np.clip(np.mean(nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
                    Rover.speed_check = 0
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        Rover.speed_check = 0
    
    return Rover

