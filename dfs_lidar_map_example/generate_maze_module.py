#!/usr/bin/env python3

import cv2
import numpy as np
""" 
string image_path
int32 diviation
float32 map_resolution
---
int32[] maze
float64[] maze_w_coordinates 
"""

""" tested with
    image_path = '/home/mrc_stn/ros2_ws/src/slam_sim/slam_sim/map_test.png'
    diviation = 2 # metre
    map_resolution = 0.05 # metre 
"""

class GenerateMap():
    def __init__(self):
        print("harita donusturuluyor")

    def pad_image(self,image, grid_size):
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
        
    def div_map(self,image, grid_size):
        
        num_rows = image.shape[0] // grid_size[0]
        num_cols = image.shape[1] // grid_size[1]

        grids = []

        for row in range(num_rows):
            for col in range(num_cols):
                grid = image[row * grid_size[0]: (row + 1) * grid_size[0],
                            col * grid_size[1]: (col + 1) * grid_size[1]]
                grids.append(grid)

        return grids,num_rows,num_cols

    def meters_to_pixels(self,diviation, map_resolution):
        #only square
        return (int(diviation * 100 * map_resolution),int(diviation * 100 * map_resolution))

    def pixels_to_meters(self,pixels, map_resolution):
        return (int(round(pixels[0] * map_resolution)),int(round(pixels[1] * map_resolution)))

    def check_middle_pixel_color(self,grid, threshold):
        
        # duzenleme gerekli
        height, width, _ = grid.shape
        middle_pixel = grid[height // 2, width // 2]
        rtrn_bool = np.all(middle_pixel > threshold) 
        return rtrn_bool

    def create_white_presence_array(self,grids, threshold):
        presence_array = []
        for grid in grids:
            is_above_threshold = self.check_middle_pixel_color(grid, threshold)
            if is_above_threshold:
                presence_array.append(0)  
            else:
                presence_array.append(1)  
        return presence_array

    def find_original_grid_centers(self,image,grids,grid_size):
        grid_centers = []  

        for i, grid in enumerate(grids):
            # Orta noktanın koordinatlarını hesapla
            center_x = (i % (image.shape[1] // grid_size[1]) + 0.5) * grid_size[1]
            center_y = (i // (image.shape[1] // grid_size[1]) + 0.5) * grid_size[0]
            grid_centers.append((center_x, center_y))
        return grid_centers
    

    def callback(self,image_path,diviation,map_resolution):
        image_path_write = '/home/mrc_stn/ros2_ws/src/custom_service_examples/custom_service_examples/wander.png'
        image = cv2.imread(image_path)

        rotated_image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        image = cv2.flip(rotated_image, 0)

        grid_size = self.meters_to_pixels(diviation, map_resolution)
        print("grid_size,",grid_size)
        cp_img = np.copy(image)
        
        #padded_image = pad_image(image, grid_size)

        grids,num_rows,num_cols = self.div_map(image, grid_size)

        grid_centers = self.find_original_grid_centers(image,grids,grid_size)
        

        grid_centers_meter = []
        
        #print(grid_centers)
        for row in grid_centers:
            print(row)
        for i in grid_centers:
            grid_centers_meter.append(self.pixels_to_meters(i,map_resolution))
            cv2.circle(cp_img, (int(i[0]),int(i[1])), 1,(255, 0, 0), 2)
        #print(grid_centers_meter)
        
        cv2.imwrite(image_path_write,cp_img)
        #print(grid_centers)
        
        #find clear grid to go to the goal_pose
        threshold = 250  
        white_presence_array = self.create_white_presence_array(grids, threshold) # maze
        maze = [white_presence_array[i:i+num_cols] for i in range(0, len(white_presence_array), num_cols)]
        maze_w_coordinates = [grid_centers_meter[i:i+num_cols] for i in range(0, len(grid_centers_meter), num_cols)]
        #print(len(white_presence_array))

        return maze,maze_w_coordinates
        
        '''
        cv2.imshow("original", image)

        while True:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
        cv2.destroyAllWindows()
        '''
