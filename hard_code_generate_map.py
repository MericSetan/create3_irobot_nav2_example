#!/usr/bin/env python3
import cv2
import numpy as np

def pad_image(image, grid_size):
    height, width, _ = image.shape

    if height % grid_size[0] != 0:
        kalan_v = height % grid_size[0]
        padd_v = grid_size[0]-kalan_v

    if width % grid_size[1] != 0:
        kalan_h = width % grid_size[1]
        padd_h = grid_size[1]-kalan_h

    new_height = height+ 2*padd_v
    new_width = width+ 2*padd_h 

    padded_image = np.zeros((new_height,new_width , 3), dtype=np.uint8)
    padded_image[padd_v:padd_v+height, padd_h:padd_h+width] = image
    
    return padded_image
    
def div_map(image, grid_size):
    
    num_rows = image.shape[0] // grid_size[0]
    num_cols = image.shape[1] // grid_size[1]

    grids = []

    for row in range(num_rows):
        for col in range(num_cols):
            grid = image[row * grid_size[0]: (row + 1) * grid_size[0],
                        col * grid_size[1]: (col + 1) * grid_size[1]]
            grids.append(grid)

    return grids

def meters_to_pixels(meters, map_resulation):
    #only square
    return (int(meters * 100 * map_resulation),int(meters * 100 * map_resulation))

def pixels_to_meters(pixels, map_resulation):
    return (pixels[0] / (100 * map_resulation), pixels[1] / (100 * map_resulation))

def check_middle_pixel_color(grid, threshold):
    height, width, _ = grid.shape
    middle_pixel = grid[height // 2, width // 2]
    rtrn_bool = np.all(middle_pixel > threshold)
    return rtrn_bool

def create_white_presence_array(grids, threshold):
    presence_array = []
    for grid in grids:
        is_above_threshold = check_middle_pixel_color(grid, threshold)
        if is_above_threshold:
            presence_array.append(1)  
        else:
            presence_array.append(0)  
    return presence_array

def find_original_grid_centers(image,grids,grid_size):
    grid_centers = []  

    for i, grid in enumerate(grids):
        # Orta noktanın koordinatlarını hesapla
        center_x = (i % (image.shape[1] // grid_size[1]) + 0.5) * grid_size[1]
        center_y = (i // (image.shape[1] // grid_size[1]) + 0.5) * grid_size[0]
        grid_centers.append((center_x, center_y))
    return grid_centers

def main():
    image_path = '/home/mrc_stn/ros2_ws/src/slam_sim/slam_sim/map_test.png'
    diviation = 2 # metre
    map_resulation = 0.05 # metre

    image = cv2.imread(image_path)

    grid_size = meters_to_pixels(diviation, map_resulation)
    
    #padded_image = pad_image(image, grid_size)

    grids = div_map(image, grid_size)

    grid_centers = find_original_grid_centers(image,grids,grid_size)
    grid_centers_meter = []
    for i in grid_centers:
        grid_centers_meter.append(pixels_to_meters(i,map_resulation))


    #find clear grid to go to the goal_pose
    threshold = 240  
    white_presence_array = create_white_presence_array(grids, threshold) # maze
    #print(white_presence_array)
    '''
    cv2.imshow("original", image)

    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
    cv2.destroyAllWindows()
    '''

if __name__=="__main__":
    main()