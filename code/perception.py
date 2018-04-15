import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def bw_thresh(img, bw_threshold_value = 150):
    
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

def rock_thresh(img):
    rocks = np.copy(img)
    
    #filter out walls
    mask = bw_thresh(img, 90) < 1
    
    rocks[mask] = 0
    
    #filter out ground  - use hsv values
    rocks = cv2.cvtColor(rocks, cv2.COLOR_BGR2HSV)
    mask = rocks[:,:,1] > 100
    
    rocks[~mask] = [0,0,0]
    rocks[mask]=[1,1,1]
    
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


def get_rock_world_coordinates(img, source, destination,xpos,ypos,yaw,world_size,scale):
    rover_perspect = np.copy(img)
    rover_perspect= cut_top_of_colored_image(rover_perspect)
    threshed = rock_thresh(rover_perspect)
    top_down = perspect_transform(threshed, source, destination)
    
    top_down = top_down[:,:,0]
    threshed = threshed[:,:,0]
    xpix, ypix = rover_coords(top_down)
    
    
    #filter distances too far
    pix_mask = xpix**2+ypix**2 <= 80**2
    xpix=xpix[pix_mask] 
    ypix=ypix[pix_mask] 
    
    #filter distances angles outside of 15deg
    pix_mask = (np.arctan2(ypix,xpix)*180/np.pi)**2 < 30**2
    xpix=xpix[pix_mask] 
    ypix=ypix[pix_mask] 

    return pix_to_world(xpix,ypix,xpos,ypos,yaw,world_size,scale)


def cut_top_of_colored_image(img, pixels_to_cut=60):
    cut_img = np.copy(img)
    cut_img[:pixels_to_cut,:] = [0,0,0]
    return cut_img

def get_obstacle_world_coordinates(img, source, destination,xpos,ypos,yaw,world_size,scale):
    threshed = bw_thresh(img)
    warped = perspect_transform(threshed, source, destination)
    warped = warped[:,:,0]
    threshed = threshed[:,:,0]
    mask = threshed > 0
    threshed = np.ones_like(threshed)
    threshed[mask]=0
    threshed = perspect_transform(threshed, source, destination)
    xpix, ypix = rover_coords(threshed)
    pix_mask = xpix**2+ypix**2 >= 80**2
    xpix[pix_mask] = 0
    ypix[pix_mask] = 0
    
    #filter distances angles outside of 15deg
    pix_mask = (np.arctan2(ypix,xpix)*180/np.pi)**2 > 30**2
    xpix[pix_mask] = 0
    ypix[pix_mask] = 0
    return pix_to_world(xpix,ypix,xpos,ypos,yaw,world_size,scale)

def get_navigible_terrain_world_coordinates(img, source, destination,xpos,ypos,yaw,world_size,scale):
    threshed = bw_thresh(cut_top_of_colored_image(img))
    warped = perspect_transform(threshed, source, destination)
    warped = warped[:,:,0]
    threshed = threshed[:,:,0]
    xpix, ypix = rover_coords(warped)
    #filter distances too far
    pix_mask = xpix**2+ypix**2 >= 80**2
    xpix[pix_mask] = 0
    ypix[pix_mask] = 0
    
    #filter distances angles outside of 15deg
    pix_mask = (np.arctan2(ypix,xpix)*180/np.pi)**2 > 30**2
    xpix[pix_mask] = 0
    ypix[pix_mask] = 0
    xpix_w, ypix_w = pix_to_world(xpix,ypix,xpos,ypos,yaw,world_size,scale)
    return xpix_w, ypix_w, threshed

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    # 2) Apply perspective transform
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles

    img = Rover.img
    dst_size = 10 
    bottom_offset = 6

    #Source values gathered above
    source = np.float32([[6.7,145.5], 
                         [306.1, 142.7],
                         [ 197.7, 96.8], 
                         [118.2, 96.7]])

    #map rover perspective to bottom middle of top down view:
    img_w = img.shape[1]
    img_h = img.shape[0]

    destination = np.float32([[img_w/2 - dst_size/2, img_h - bottom_offset],
                  [img_w/2 + dst_size/2, img_h - bottom_offset],
                  [img_w/2 + dst_size/2, img_h - dst_size - bottom_offset], 
                  [img_w/2 - dst_size/2, img_h - dst_size - bottom_offset],
                  ])
    navigable_x_world,navigable_y_world, threshed = get_navigible_terrain_world_coordinates(np.copy(img),\
                                                                                  source, \
                                                                                  destination,\
                                                                                  Rover.pos[0],\
                                                                                  Rover.pos[1],\
                                                                                  Rover.yaw,\
                                                                                  200,\
                                                                                  10)

    rock_x_world, rock_y_world = get_rock_world_coordinates(np.copy(img),\
                                                          source, \
                                                          destination,\
                                                          Rover.pos[0],\
                                                          Rover.pos[1],\
                                                          Rover.yaw,\
                                                          200,\
                                                          10)
    
    obstacle_x_world, obstacle_y_world = get_obstacle_world_coordinates(np.copy(img),\
                                                                      source, \
                                                                      destination,\
                                                                      Rover.pos[0],\
                                                                      Rover.pos[1],\
                                                                      Rover.yaw,\
                                                                      200,\
                                                                      10)
        
        
    Rover.map_count[obstacle_y_world, obstacle_x_world, 0] += 1
    Rover.map_count[navigable_y_world, navigable_x_world, 1] += 1
    
    obstacle = (Rover.map_count[:, :, 0] > Rover.map_count[:, :, 1])  &  (Rover.worldmap[:, :, 1] < 1)
    naviagable = (Rover.map_count[:, :, 1] > Rover.map_count[:, :, 0]) &  (Rover.worldmap[:, :, 1] < 1)
    Rover.worldmap[obstacle, 0] = 255
    Rover.worldmap[obstacle, 1] = 0
    Rover.worldmap[obstacle, 2] = 0

    Rover.worldmap[naviagable, 0] = 0
    Rover.worldmap[naviagable, 1] = 0
    Rover.worldmap[naviagable, 2] = 255
    
    Rover.worldmap[rock_y_world, rock_x_world, 1] = 255
    Rover.worldmap[rock_y_world, rock_x_world, 0] = 0
    Rover.worldmap[rock_y_world, rock_x_world, 2] = 0
    
    return Rover
