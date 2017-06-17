import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        # For exploration we do not need to get close to non-navigable terrain
        # therefore, set a higher value here. this helps staying in safe terrain
        Rover.stop_forward = 400
        # trim area of navigable terrain to be close to center. It's used when turning in place
        trim_angle = 0.001
        nav_angles = Rover.nav_angles[np.where(np.abs(Rover.nav_angles) < trim_angle)]
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            # Naive greedy implementation for sample. As soon as on is detected go for it
            if Rover.sample_bearing is not None:
                Rover.steer = Rover.sample_bearing * 180 / np.pi
                # at a distance reduce speed
                s_dist = 60
                if Rover.sample_dist < s_dist:
                    Rover.throttle = 0.02
                # when close stop
                if Rover.near_sample:
                    print('stopping to pickup sample')
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = Rover.sample_bearing * 180 / np.pi
                    Rover.mode = 'stop'
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
                # bias the angle to the left to hug to the left wall
                wall_bias = 5
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15) + wall_bias
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(nav_angles) < Rover.stop_forward:
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
                if (len(nav_angles) < Rover.go_forward) and not Rover.near_sample:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -5 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(nav_angles) >= Rover.go_forward and not Rover.near_sample:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

