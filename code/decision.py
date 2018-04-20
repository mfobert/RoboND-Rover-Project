import numpy as np

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function

#if the rovers naviagable path values are greater than the threshold for a clear path,
#we can assume there is a clear path ahead. #NOTE: this is not always true, such as in
#the case that there is a small rock in the Rover's path; need to implement a seperate
#function that detects if the rover is stuck in one place for too long to take care of
#these issues
def clear_path(Rover):
    if Rover.nav_dists.mean() > Rover.clear_path:
        return True
    else:
        return False

#make the rover go forward - if maximum velocity is met, coast instead of accelerating
def go_forward(Rover):
    Rover.brake = 0
    if Rover.vel < Rover.max_vel:
        # Set throttle value to throttle setting
        Rover.throttle = Rover.throttle_set
    else: # Else coast
        Rover.throttle = 0

    #steer in the direction of the greatest average navigable path
    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi)*Rover.aggresive_steering_amplitude, -15, 15)
    
    if not Rover.wall_on_left:
        #if the wall is not seen on the left of the rover, aggressively steer in it's direction
        Rover.steer = 10

def sqr_distance(x1,y1,x2,y2):
    return (x1-x2)**2 + (y1-y2)**2

def stuck(Rover):
    #every so often, check if the position of the rover has changed in
    #a substantial way - otherwise we are stuck.
    if  Rover.total_time - Rover.rover_stuck_check_time_last_checked > Rover.rover_stuck_check_interval:
        #if its time to check if we're stuck
        Rover.rover_stuck_check_time_last_checked = Rover.total_time

        #compare the distance to the last time checked
        if (Rover.rover_stuck_check_distance_threshold**2 < \
            sqr_distance(Rover.pos[0], Rover.pos[1], \
                         Rover.rover_stuck_check_last_x, Rover.rover_stuck_check_last_y)):
                #rover is not stuck, update values and return false
                Rover.rover_stuck_check_last_x = Rover.pos[0]
                Rover.rover_stuck_check_last_y = Rover.pos[1]
        else:
            #Rover is stuck
            Rover.rover_stuck_yaw = Rover.yaw #remember stuck yaw for future processing
            Rover.mode = "Stuck"
            stop(Rover)

def found_rock(Rover):
    if len(Rover.rock_angles) > Rover.rock_thresh:
        Rover.mode = "ROCK"
        Rover.rock_mode_stage = 0
        Rover.pos_when_finding_rock = Rover.pos
        Rover.yaw_when_finding_rock = Rover.yaw
        stop(Rover)
        Rover.time_rock = Rover.total_time

#Stop the rover...
def stop(Rover):
    Rover.throttle = 0
    # Set brake to stored brake value
    Rover.brake = Rover.brake_set
    Rover.steer = 0

#make decisions based on the Rover's perception/telemetry on how to navigate the terrain.
#this Rovers algorithm leads it to follow the left wall (once found) around the map.
def decision_step(Rover):

    #if our data is good
    if Rover.nav_angles is not None:
        
        if Rover.mode == "Find Wall":
            #at the start we need to find a wall to follow in the first place
            #drive forward until wall aquired.
            go_forward(Rover)
            if Rover.wall_on_left:
                #found the wall, now follow it
                Rover.mode = "Follow Wall"
                stop(Rover)
        elif Rover.mode == "Follow Wall":
            #Once we have found the wall, follow it always with it on the left

            #check if rover is stuck
            stuck(Rover)
                
            if Rover.time_without_seeing_wall > Rover.time_lost_wall_threshold:
                #Rover lost wall - look for it, it will be on the left.
                stop(Rover)
                Rover.mode = "Lost wall"
            else:
                #The rover has not lost the wall (for a determined amount of time atleast)
                
                if not Rover.wall_on_left:
                    #if the rover does not see the wall right now, add to the time that is hasn't seen
                    #the wall
                    Rover.time_without_seeing_wall += Rover.total_time-Rover.time_last
                else:
                    #if we see the wall on the left, make the time we havent seen the wall 0
                    Rover.time_without_seeing_wall = 0.0
                
                if clear_path(Rover):
                    #if there is a clear path forwards, go that way
                    go_forward(Rover)
                    Rover.time_mean_distance_less_than_thresh=0
                    found_rock(Rover) #check if we see a rock
                else:
                    #if we have a disrupted path, add to the amount of time this has been happening
                    Rover.time_mean_distance_less_than_thresh += Rover.total_time-Rover.time_last

                    if Rover.time_mean_distance_less_than_thresh > Rover.max_time_mean_distance_less_than_thresh:
                        #if we have had a disrupted path for too long, stop and proceed accordingly. 
                        stop(Rover)
                        Rover.mode = "Disrupted Path"

                #remember the last time we made these decisions - the frame rate
                Rover.time_last = Rover.total_time
                    
        elif Rover.mode == "Lost wall":
            #the rover has lost the wall
            
            if Rover.vel > 0:
                #stop the rover
                stop(Rover)
            else:
                if Rover.wall_on_left:
                    #found the wall, proceed accordinly.
                    Rover.mode = "Follow Wall"
                    Rover.time_without_seeing_wall = 0.0
                    Rover.time_last = Rover.total_time
                else:
                    #steer left until we find the wall
                    Rover.brake = 0
                    Rover.steer = 15
        elif Rover.mode == "Disrupted Path":
            # we have obstacles in our path
            if Rover.vel > 0:
                #stop the rover
                stop(Rover)
            else:
                if clear_path(Rover):
                    #clear path, proceed accordinly
                    Rover.mode = "Follow Wall"
                    Rover.time_mean_distance_less_than_thresh = 0
                    Rover.time_last = Rover.total_time
                else:
                    #steer right until clear path
                    Rover.brake = 0
                    Rover.steer = -15
        elif Rover.mode == "Stuck":
                Rover.send_pickup = False
                #turn right atleast 90 degrees
                if (Rover.yaw - Rover.rover_stuck_yaw)**2 > 90**2:
                    #90 degrees accomplished
                    Rover.mode = "Follow Wall"
                else:
                    Rover.throttle = 0
                    Rover.steer = -15
        elif Rover.mode == "ROCK":
            if Rover.total_time - Rover.time_rock < Rover.time_rock_max:
                if not Rover.near_sample:
                    #stop the rover
                    Rover.brake = 0
                    Rover.throttle = 0.1
                    if len(Rover.rock_angles) >0:
                        Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
                    else:
                        Rover.steer = 0
                else:
                    if Rover.vel > 0.01:
                        stop(Rover)
                    else:
                        Rover.brake = 0
                        Rover.send_pickup = True
                        Rover.rover_stuck_yaw = Rover.yaw
                        Rover.mode = "Stuck"
            else:
                Rover.rover_stuck_yaw = Rover.yaw
                Rover.mode = "Stuck"
                
            #except:
            #    Rover.mode = "Follow Wall"
            
                    
        
            
    else:
        #data not good. Throw an error and do nothing.
        Rover.mode = "Error"
        Rover.throttle = 0
        Rover.steer = 0
        Rover.brake = 0
        

        
    
    return Rover

