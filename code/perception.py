import numpy as np
import cv2

#threshold a black and white image - used to filter out walls from navigable path for the rover
#returns 3 channel image such that it can be transformed properly
def bw_thresh(img, bw_threshold_value = 160):
    
    #convert to black and white with opencv
    img_bw = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    
    # Create an array of zeros same xy size as img
    color_select = np.zeros_like(img)
    
    #mask based on threshold
    mask = img_bw >= bw_threshold_value
    
    # Index the array of zeros with the boolean array and set to 1
    color_select[mask] = [1,1,1]
    color_select[~mask] = [0,0,0]
    # Return the binary image
    return color_select

#filter out the yellow rocks from the image to locate them
#returns 3 channel image such that it can be transformed properly
def rock_thresh(img):
    #copy the input image such that is is not edited by reference
    rocks = np.copy(img)
    
    #filter out walls (not very aggressively)
    mask = bw_thresh(img, 90) < 1

    rocks[mask] = 0
    
    #using HSV values, filter out the ground and everything except the rocks
    rocks = cv2.cvtColor(rocks, cv2.COLOR_BGR2HSV) #convert to HSV
    mask = rocks[:,:,1] > 100 #if the 2nd channel is greater than 100, the pixel is a rock
    
    rocks[~mask] = [0,0,0] #all of the values not at the mask are not rocks (0)
    rocks[mask]=[1,1,1] #all of the values at the mask are not rocks (1)
    
    return rocks


# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
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

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

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
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped

#takes a picture in from the rover's perspective and
#returns rock coordinates in the world view perspective
def get_rock_world_coordinates(img, source, destination,xpos,ypos,yaw,world_size,scale):
    #copy input image
    rover_perspect = np.copy(img)
    #cut out the sky
    rover_perspect= cut_top_of_colored_image(rover_perspect)
    #threshold the image for rocks
    threshed = rock_thresh(rover_perspect)
    #convert this into top down pixel perspective
    top_down = perspect_transform(threshed, source, destination)

    #convert to one channel image
    top_down = top_down[:,:,0]
    threshed = threshed[:,:,0]

    #conver to rover centric coordinates
    xpix, ypix = rover_coords(top_down)
    
    #filter distances too far from the rover to be considered accurate
    pix_mask = xpix**2+ypix**2 <= 80**2
    xpix=xpix[pix_mask] 
    ypix=ypix[pix_mask] 
    
    #filter out angles outside of 30deg
    pix_mask = (np.arctan2(ypix,xpix)*180/np.pi)**2 < 30**2
    xpix=xpix[pix_mask] 
    ypix=ypix[pix_mask] 

    #return the world coordinates of the rocks and rover polar coordinates
    x_pix_world,y_pix_world =pix_to_world(xpix,ypix,xpos,ypos,yaw,world_size,scale)

    #get the polar values
    dist, angles= to_polar_coords(xpix, ypix)
    
    return x_pix_world,y_pix_world,dist, angles

#cuts off the top of a colored image. Can be used to cut out the sky
#from most of the perception analysis
def cut_top_of_colored_image(img, pixels_to_cut=60):
    cut_img = np.copy(img)
    cut_img[:pixels_to_cut,:] = [0,0,0]
    return cut_img


#given an image taken from the rover's perspective,
#return the position of obstacles in the world-view perspective
def get_obstacle_world_coordinates(img, source, destination,xpos,ypos,yaw,world_size,scale):
    #threshold the image for navigable terrain
    threshed = bw_thresh(img)

    #change to a top-down perspective
    #warped = perspect_transform(threshed, source, destination)

    #convert image to 1 channel
    #warped = warped[:,:,0]
    threshed = threshed[:,:,0]

    #any values in the threshed image greater than 0 are considered naviagble
    #turn these values to 0, as we want the unnavigable terrain here
    mask = threshed > 0
    threshed = np.ones_like(threshed)
    threshed[mask]=0

    #cahange to a top down perspective
    threshed = perspect_transform(threshed, source, destination)

    #get rover-centric coordiantes of these obstacles
    xpix, ypix = rover_coords(threshed)

    #anything further away than 80 consider to be innaccurate and neglect
    pix_mask = xpix**2+ypix**2 >= 80**2
    xpix[pix_mask] = 0
    ypix[pix_mask] = 0
    
    #filter out values with angles outside of 30deg
    pix_mask = (np.arctan2(ypix,xpix)*180/np.pi)**2 > 30**2
    xpix[pix_mask] = 0
    ypix[pix_mask] = 0

    #get the polar values
    dist, angles= to_polar_coords(xpix, ypix)

    #get the world coordinate values
    xpix_w, ypix_w = pix_to_world(xpix,ypix,xpos,ypos,yaw,world_size,scale)
    return xpix_w, ypix_w ,dist, angles

def get_navigible_terrain_world_coordinates(img, source, destination,xpos,ypos,yaw,world_size,scale):
    #threshold the image for navigable terrain
    threshed = bw_thresh(cut_top_of_colored_image(img))
    #change to a top-down perspective
    warped = perspect_transform(threshed, source, destination)

    #convert to one channel images
    warped = warped[:,:,0]
    threshed = threshed[:,:,0]

    #convert to rover centric coordinates
    xpix, ypix = rover_coords(warped)
    
    #anything further away than 80 consider to be innaccurate and neglect
    pix_mask = xpix**2+ypix**2 >= 80**2
    xpix[pix_mask] = 0
    ypix[pix_mask] = 0
    
    #filter distances angles outside of 30deg
    pix_mask = (np.arctan2(ypix,xpix)*180/np.pi)**2 > 30**2
    xpix[pix_mask] = 0
    ypix[pix_mask] = 0
    xpix_w, ypix_w = pix_to_world(xpix,ypix,xpos,ypos,yaw,world_size,scale)

    #get polar coordinate rover centric navigable terrain distances and angles
    dist, angles= to_polar_coords(xpix, ypix)

    return xpix_w, ypix_w, dist, angles


#sets the Rover boolean value Rover.wall_on_left depending on
#if there are enough (threshold dependent) obstacles at an angle greater than
#10 degrees to the left of the rover
def wall_on_left_set(obstacles_rover_polar_angles, Rover):
    #get the obstacles to the left of the rover (more than 10 degrees)
    wall_on_left_mask = obstacles_rover_polar_angles*180/np.pi > 10

    #count them
    wall_count = np.zeros_like(wall_on_left_mask)
    wall_count[wall_on_left_mask] = 1
    Rover.wall_left_amount = np.count_nonzero(wall_count)

    #if greater than threshold, then there is a wall on the left of the rover
    if np.count_nonzero(wall_count) > Rover.wall_on_left_threshold_pix:
        Rover.wall_on_left = True
    else:
        Rover.wall_on_left = False
    

def perception_step(Rover):
    # Perform perception steps to update Rover()

    #get the rovers input image
    img = Rover.img

    #
    #Set perspective transform values
    #
    
    dst_size = 10 #perspective transform scale
    bottom_offset = 6 #perspective transform offset

    #perspecive transform grid corner source
    source = np.float32([[6.7,145.5], 
                         [306.1, 142.7],
                         [ 197.7, 96.8], 
                         [118.2, 96.7]])
   
    #perspecive transform grid corner destination
     #map rover perspective to bottom middle of top down view:
    img_w = img.shape[1]
    img_h = img.shape[0]
    destination = np.float32([[img_w/2 - dst_size/2, img_h - bottom_offset],
                  [img_w/2 + dst_size/2, img_h - bottom_offset],
                  [img_w/2 + dst_size/2, img_h - dst_size - bottom_offset], 
                  [img_w/2 - dst_size/2, img_h - dst_size - bottom_offset],
                  ])


    #percieve navigable terrain data in xy-world/polar-rover-centric coordiantes
    navigable_x_world,navigable_y_world, \
    rover_centric_pixel_distances, \
    rover_centric_angles =  \
            get_navigible_terrain_world_coordinates(np.copy(img),\
                                                          source, \
                                                          destination,\
                                                          Rover.pos[0],\
                                                          Rover.pos[1],\
                                                          Rover.yaw,\
                                                          200,\
                                                          10)
    
    #percieve rock data in xy-world coordinates
    rock_x_world, \
    rock_y_world, \
    Rover.rock_distances, \
    Rover.rock_angles = get_rock_world_coordinates(np.copy(img),\
                                                          source, \
                                                          destination,\
                                                          Rover.pos[0],\
                                                          Rover.pos[1],\
                                                          Rover.yaw,\
                                                          200,\
                                                          10)
    
    #percieve obstacle data in xy-world/polar-rover-centric coordiantes
    obstacle_x_world, \
    obstacle_y_world, \
    dist_obstacles_rover,\
    angles_obstacles_rover = \
        get_obstacle_world_coordinates(np.copy(img),\
                                          source, \
                                          destination,\
                                          Rover.pos[0],\
                                          Rover.pos[1],\
                                          Rover.yaw,\
                                          200,\
                                          10)

    #determine if there is a wall on the left of the rover
    wall_on_left_set(angles_obstacles_rover, Rover)

    #
    #Build/Adjust world map
    #

    #count how many times we have seen an obstacle (0) at an XY position VS. navigable terrain (1)
    Rover.map_count[obstacle_y_world, obstacle_x_world, 0] += 1
    Rover.map_count[navigable_y_world, navigable_x_world, 1] += 1

    #if we have seen more obstacles than naviagable terrain at a position, set the world map as an obstacle
    obstacle = (Rover.map_count[:, :, 0] > Rover.map_count[:, :, 1])  &  (Rover.worldmap[:, :, 1] < 1)
    #otherwise set it as navigable
    naviagable = (Rover.map_count[:, :, 1] > Rover.map_count[:, :, 0]) &  (Rover.worldmap[:, :, 1] < 1)

    #Obstacles are red (255,0,0)
    Rover.worldmap[obstacle, 0] = 255
    Rover.worldmap[obstacle, 1] = 0
    Rover.worldmap[obstacle, 2] = 0

    #naviagable terrain is blue (0,0,255)
    Rover.worldmap[naviagable, 0] = 0
    Rover.worldmap[naviagable, 1] = 0
    Rover.worldmap[naviagable, 2] = 255


    #Rocks are green (0,255,0)
    Rover.worldmap[rock_y_world, rock_x_world, 0] = 0
    Rover.worldmap[rock_y_world, rock_x_world, 1] = 255
    Rover.worldmap[rock_y_world, rock_x_world, 2] = 0
    
    #set the rover polar coordinate data to be used later in decision making
    Rover.nav_dists = rover_centric_pixel_distances
    Rover.nav_angles = rover_centric_angles
    
    return Rover
