import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function

def clear_path(Rover):
    if Rover.nav_dists.mean() > Rover.clear_path:
        return True
    else:
        return False

def go_forward(Rover):
    Rover.brake = 0
    if Rover.vel < Rover.max_vel:
        # Set throttle value to throttle setting
        Rover.throttle = Rover.throttle_set
    else: # Else coast
        Rover.throttle = 0
    
    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi)*Rover.aggresive_steering_amplitude, -15, 15)
    if not Rover.wall_on_left:
        Rover.steer = 15

def stop(Rover):
    Rover.throttle = 0
    # Set brake to stored brake value
    Rover.brake = Rover.brake_set
    Rover.steer = 0

def decision_step(Rover):


    if Rover.nav_angles is not None:
        if Rover.mode == "Find Wall":
            #drive forward until wall aquired.
            go_forward(Rover)
            if Rover.wall_on_left:
                Rover.mode = "Follow Wall"
                stop(Rover)
        elif Rover.mode == "Follow Wall":
            #Rover lost wall
            if Rover.time_without_seeing_wall > Rover.time_lost_wall_threshold:
                stop(Rover)
                Rover.mode = "Lost wall"
            else:
                if not Rover.wall_on_left:
                    Rover.time_without_seeing_wall += Rover.total_time-Rover.time_last
                else:
                    Rover.time_without_seeing_wall = 0.0
                
                if clear_path(Rover):
                    go_forward(Rover)
                    Rover.time_mean_distance_less_than_thresh=0
                else:
                    Rover.time_mean_distance_less_than_thresh += Rover.total_time-Rover.time_last
                    if Rover.time_mean_distance_less_than_thresh > Rover.max_time_mean_distance_less_than_thresh:
                        stop(Rover)
                        Rover.mode = "Disrupted Path"

                Rover.time_last = Rover.total_time
                    
        elif Rover.mode == "Lost wall":
            if Rover.vel > 0.1:
                stop(Rover)
            else:
                stop(Rover)
                if Rover.wall_on_left:
                    Rover.mode = "Follow Wall"
                    Rover.time_without_seeing_wall = 0.0
                    Rover.time_last = Rover.total_time
                else:
                    #steer left
                    Rover.brake = 0
                    Rover.steer = 15
        elif Rover.mode == "Disrupted Path":
            if Rover.vel > 0.1:
                stop(Rover)
            else:
                stop(Rover)
                if clear_path(Rover):
                    Rover.mode = "Follow Wall"
                    Rover.time_mean_distance_less_than_thresh = 0
                    Rover.time_last = Rover.total_time
                else:
                    #steer right
                    Rover.brake = 0
                    Rover.steer = -15
        
            
    else:
        Rover.mode = "Error"
        Rover.throttle = 9
        Rover.steer = 0
        Rover.brake = 0

        
    
    return Rover

